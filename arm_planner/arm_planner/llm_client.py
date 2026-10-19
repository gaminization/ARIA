#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Client — Async Ollama/Wrappers with Gemini Offloading
Handles: model loading/unloading, structured JSON output,
function calling schema, timeout/retry, fallback detection.
Supports two modes:
  - local:  Ollama running on localhost:11434 (default, works without GPU)
  - gemini: Google Gemini API (requires GEMINI_API_KEY in config)
════════════════════════════════════════════════════════════════
"""
import asyncio
import json
import logging
import os
import subprocess
import time
from typing import Any, Dict, List, Literal, Optional, Tuple

import aiohttp

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# Configuration
# ════════════════════════════════════════════════════════════════
def _find_config_path() -> str:
    """Locate llm_config.yaml (or .py) in source tree, ament share dir, or ARIA_ROOT."""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(os.path.dirname(here), 'config', 'llm_config.yaml'),
        os.path.join(os.path.dirname(here), 'config', 'llm_config.py'),
    ]
    try:
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('arm_planner'), 'config', 'llm_config.yaml'))
        candidates.append(os.path.join(
            get_package_share_directory('arm_planner'), 'config', 'llm_config.py'))
    except Exception:
        pass
    candidates.append(os.path.join(
        os.environ.get('ARIA_ROOT', '/home/gaminizer/Projects/ARIA'),
        'arm_planner', 'config', 'llm_config.yaml'))
    candidates.append(os.path.join(
        os.environ.get('ARIA_ROOT', '/home/gaminizer/Projects/ARIA'),
        'arm_planner', 'config', 'llm_config.py'))
    for c in candidates:
        if os.path.exists(c):
            return c
    return candidates[0]


_CONFIG_PATH = _find_config_path()


def _load_config() -> dict:
    """Load LLM configuration from YAML or Python config file."""
    if _CONFIG_PATH.endswith('.yaml') or _CONFIG_PATH.endswith('.yml'):
        import yaml
        try:
            with open(_CONFIG_PATH, 'r') as f:
                return yaml.safe_load(f) or {}
        except FileNotFoundError:
            logger.warning(f"{_CONFIG_PATH} not found, using defaults")
            return {}
    else:
        # Python config file
        try:
            with open(_CONFIG_PATH, 'r') as f:
                exec_globals = {}
                exec(f.read(), exec_globals)
                return exec_globals
        except FileNotFoundError:
            logger.warning(f"{_CONFIG_PATH} not found, using defaults")
            return {}


# ── Runtime mode detection ────────────────────────────────────
def _detect_mode(config: dict) -> Literal["local", "gemini"]:
    """Determine the LLM mode from config flags."""
    enable = config.get('ENABLE_GEMINI', 'auto').lower()
    key = config.get('GEMINI_API_KEY', '')
    if enable == 'true':
        return 'gemini'
    if enable == 'false':
        return 'local'
    # auto: use Gemini if key available, else local
    return 'gemini' if key else 'local'


# ═══════════════════════════════════════════════════════════════
# Model Registry
# ════════════════════════════════════════════════════════════════
SUPPORTED_MODELS: Dict[str, Dict[str, Any]] = {
    "llama3.1:8b-instruct-q4_K_M": {
        "vram_gb": 5.0,
        "context_length": 128_000,
        "supports_vision": False,
        "use_for": ["planning", "reasoning", "dialogue"],
    },
    "qwen2.5-vl:7b-instruct-q4_K_M": {
        "vram_gb": 5.2,
        "context_length": 32_768,
        "supports_vision": True,
        "use_for": ["visual_planning", "scene_description"],
    },
    "llama3.1:8b-instruct-q8_0": {
        "vram_gb": 8.5,
        "context_length": 128_000,
        "supports_vision": False,
        "use_for": ["planning_high_quality"],
    },
}


# ═══════════════════════════════════════════════════════════════
# OllamaClient
# ════════════════════════════════════════════════════════════════
class OllamaClient:
    """
    Async wrapper around Ollama local inference server with Gemini offloading.

    Modes:
      - local:   Send requests to http://localhost:11434 (Ollama)
      - gemini:  Send requests to Google Gemini API

    Features:
      - Lazy model loading with VRAM headroom checks
      - Explicit unloading to free VRAM during execution
      - Structured JSON mode with validation and retry
      - Health checking with fast timeout
      - Automatic mode switching based on config
    """

    def __init__(
        self,
        base_url: str = "http://localhost:11434",
        config: Optional[dict] = None,
    ):
        self.base_url = base_url.rstrip('/')
        self.config = config or _load_config()
        self._loaded_model: Optional[str] = None
        self._session: Optional[aiohttp.ClientSession] = None
        self._load_lock = asyncio.Lock()

        # Detect mode and set up accordingly
        self.mode = _detect_mode(self.config)
        self._gemini_key = self.config.get('GEMINI_API_KEY', '')
        self._gemini_endpoint = self.config.get(
            'GEMINI_ENDPOINT',
            'https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent')

        # Config values with defaults
        self.default_model = self.config.get(
            'preferred_model', 'llama3.1:8b-instruct-q4_K_M')
        self.default_temperature = self.config.get('temperature', 0.1)
        self.default_timeout = self.config.get('timeout_s', 30.0)
        self.default_retries = self.config.get('max_retries', 2)
        self.vram_headroom = self.config.get('vram_headroom_gb', 2.5)
        self.unload_after_planning = self.config.get(
            'unload_after_planning', True)

        logger.info(f"LLM mode: {self.mode} (local={self.mode == 'local'}, "
                    f"gemini_key_set={bool(self._gemini_key)})")

    # ── Session management ─────────────────────────────────
    async def _get_session(self) -> aiohttp.ClientSession:
        """Get or create aiohttp session."""
        if self._session is None or self._session.closed:
            timeout = aiohttp.ClientTimeout(total=120)
            self._session = aiohttp.ClientSession(timeout=timeout)
        return self._session

    async def close(self):
        """Close the aiohttp session."""
        if self._session and not self._session.closed:
            await self._session.close()
            self._session = None

    # ── Health Check ───────────────────────────────────────
    async def is_available(self) -> bool:
        """
        Quick health check: for local, GET /api/tags;
        for gemini, check if API key is configured.
        Fast timeout: 2 seconds.
        """
        if self.mode == 'local':
            try:
                session = await self._get_session()
                async with session.get(
                    f"{self.base_url}/api/tags",
                    timeout=aiohttp.ClientTimeout(total=2.0),
                ) as resp:
                    return resp.status == 200
            except (aiohttp.ClientError, asyncio.TimeoutError, OSError):
                return False
        else:  # gemini
            return bool(self._gemini_key)

    async def list_local_models(self) -> List[str]:
        """List all models available locally in Ollama (local mode only)."""
        if self.mode != 'local':
            return []
        try:
            session = await self._get_session()
            async with session.get(
                f"{self.base_url}/api/tags",
                timeout=aiohttp.ClientTimeout(total=5.0),
            ) as resp:
                if resp.status != 200:
                    return []
                data = await resp.json()
                return [m['name'] for m in data.get('models', [])]
        except (aiohttp.ClientError, asyncio.TimeoutError, OSError):
            return []

    async def get_running_models(self) -> List[str]:
        """List models currently loaded in memory via Ollama /api/ps (local mode only)."""
        if self.mode != 'local':
            return []
        try:
            session = await self._get_session()
            async with session.get(
                f"{self.base_url}/api/ps",
                timeout=aiohttp.ClientTimeout(total=5.0),
            ) as resp:
                if resp.status != 200:
                    return []
                data = await resp.json()
                return [m.get('name', '') for m in data.get('models', [])]
        except (aiohttp.ClientError, asyncio.TimeoutError, OSError):
            return []

    # ── VRAM Management ────────────────────────────────────
    async def get_vram_usage(self) -> Dict[str, float]:
        """
        Returns current VRAM usage info via nvidia-smi (local mode only).

        Returns a dict with total/used/free GB. Returns zeros if nvidia-smi
        not available (e.g. no GPU in container).
        """
        if self.mode != 'local':
            return {"total_gb": 0, "used_gb": 0, "free_gb": 0}
        try:
            loop = asyncio.get_event_loop()
            result = await loop.run_in_executor(
                None,
                lambda: subprocess.run(
                    ['nvidia-smi',
                     '--query-gpu=memory.total,memory.used,memory.free',
                     '--format=csv,nounits,noheader'],
                    capture_output=True, text=True, timeout=5,
                ),
            )
            if result.returncode != 0:
                return {"total_gb": 0, "used_gb": 0, "free_gb": 0}

            line = result.stdout.strip().split('\n')[0]
            total_mb, used_mb, free_mb = [
                float(x.strip()) for x in line.split(',')]
            return {
                "total_gb": total_mb / 1024,
                "used_gb": used_mb / 1024,
                "free_gb": free_mb / 1024,
            }
        except (subprocess.TimeoutExpired, FileNotFoundError, ValueError):
            return {"total_gb": 0, "used_gb": 0, "free_gb": 0}

    async def _check_vram_headroom(self, model_name: str) -> bool:
        """
        Check if there is enough VRAM headroom to load a model (local mode only).
        Returns True if safe to proceed.
        """
        if self.mode != 'local':
            # In gemini mode, we don't check VRAM — always allow
            return True

        running = await self.get_running_models()
        if any(model_name in r or r in model_name for r in running):
            return True

        model_info = SUPPORTED_MODELS.get(model_name)
        if not model_info:
            logger.warning(f"Unknown model '{model_name}', skipping VRAM check")
            return True

        vram = await self.get_vram_usage()
        if vram["total_gb"] == 0:
            # nvidia-smi not available — assume OK
            return True

        required = model_info["vram_gb"]
        available = vram["free_gb"]

        if available < required:
            logger.warning(
                f"VRAM tight: {available:.1f}GB free, "
                f"model needs {required:.1f}GB. "
                f"Headroom threshold: {self.vram_headroom:.1f}GB")
            # Allow loading with partial GPU offload — Ollama manages CPU/GPU split
            return available >= self.vram_headroom

        logger.info(
            f"VRAM OK: {available:.1f}GB free, "
            f"model needs {required:.1f}GB")
        return True

    # ── Model Loading / Unloading ──────────────────────────
    async def load_model(self, model_name: str) -> bool:
        """
        Explicitly pre-load model into VRAM (local mode only).

        Uses Ollama /api/generate with an empty prompt and
        keep_alive=-1 to load the model persistently.
        Checks available VRAM first.

        Returns True if loaded successfully.
        """
        async with self._load_lock:
            if self._loaded_model == model_name:
                logger.info(f"Model '{model_name}' already loaded")
                return True

            # If model is already active in Ollama, record it and succeed
            running = await self.get_running_models()
            if any(model_name in r or r in model_name for r in running):
                self._loaded_model = model_name
                logger.info(f"Model '{model_name}' already active in Ollama VRAM")
                return True

            # Check VRAM
            if not await self._check_vram_headroom(model_name):
                logger.error(
                    f"Insufficient VRAM to load '{model_name}'")
                return False

            # Unload previous model if different
            if self._loaded_model:
                await self._unload_current()

            # Load via empty generate call with keep_alive
            try:
                session = await self._get_session()
                payload = {
                    "model": model_name,
                    "prompt": "",
                    "keep_alive": -1,  # Keep loaded indefinitely
                }
                async with session.post(
                    f"{self.base_url}/api/generate",
                    json=payload,
                    timeout=aiohttp.ClientTimeout(total=120),
                ) as resp:
                    if resp.status == 200:
                        # Consume the response stream
                        async for _ in resp.content:
                            pass
                        self._loaded_model = model_name
                        logger.info(f"Model '{model_name}' loaded into VRAM")
                        return True
                    else:
                        body = await resp.text()
                        logger.error(
                            f"Failed to load model: {resp.status} — {body}")
                        return False
            except (aiohttp.ClientError, asyncio.TimeoutError) as e:
                logger.error(f"Error loading model '{model_name}': {e}")
                return False

    async def unload_model(self, model_name: Optional[str] = None) -> bool:
        """
        Explicitly unload model to free VRAM (local mode only).

        Uses keep_alive=0 to force immediate unload.
        Called after planning is complete and execution begins.
        """
        if self.mode != 'local':
            # In gemini mode, there's nothing to unload from VRAM
            logger.info("Gemini mode: nothing to unload from VRAM")
            return True

        target = model_name or self._loaded_model
        if not target:
            return True

        try:
            session = await self._get_session()
            payload = {
                "model": target,
                "prompt": "",
                "keep_alive": 0,  # Force immediate unload
            }
            async with session.post(
                f"{self.base_url}/api/generate",
                json=payload,
                timeout=aiohttp.ClientTimeout(total=15),
            ) as resp:
                # Consume stream
                async for _ in resp.content:
                    pass
                if self._loaded_model == target:
                    self._loaded_model = None
                logger.info(f"Model '{target}' unloaded from VRAM")
                return True
        except (aiohttp.ClientError, asyncio.TimeoutError) as e:
            logger.error(f"Error unloading model '{target}': {e}")
            return False

    async def _unload_current(self):
        """Unload the currently loaded model."""
        if self._loaded_model:
            await self.unload_model(self._loaded_model)

    # ── Generation ─────────────────────────────────────────
    async def generate(
        self,
        prompt: str,
        model: Optional[str] = None,
        system_prompt: str = "",
        images: Optional[List[str]] = None,
        response_format: Literal["text", "json"] = "text",
        temperature: Optional[float] = None,
        timeout_s: Optional[float] = None,
        max_retries: Optional[int] = None,
    ) -> Tuple[str, bool]:
        """
        Generate a response from the LLM.

        Args:
            prompt:          User prompt
            model:           Model name (defaults to config preferred)
            system_prompt:   System instructions
            images:          Base64-encoded images (for vision models)
            response_format: "text" or "json" (strict JSON mode)
            temperature:     Sampling temperature (low = deterministic)
            timeout_s:       Request timeout in seconds
            max_retries:     Number of retries on failure

        Returns:
            (response_text, success)
            On failure: ("", False) — caller handles fallback.
        """
        model = model or self.default_model
        temperature = temperature if temperature is not None else self.default_temperature
        timeout_s = timeout_s or self.default_timeout
        max_retries = max_retries if max_retries is not None else self.default_retries

        if self.mode == 'gemini':
            return await self._generate_gemini(
                prompt, model, system_prompt, images,
                response_format, temperature, timeout_s, max_retries)
        else:
            return await self._generate_ollama(
                prompt, model, system_prompt, images,
                response_format, temperature, timeout_s, max_retries)

    async def _generate_ollama(
        self,
        prompt: str,
        model: Optional[str] = None,
        system_prompt: str = "",
        images: Optional[List[str]] = None,
        response_format: Literal["text", "json"] = "text",
        temperature: Optional[float] = None,
        timeout_s: Optional[float] = None,
        max_retries: Optional[int] = None,
    ) -> Tuple[str, bool]:
        """Generate via local Ollama server."""
        model = model or self.default_model
        temperature = temperature if temperature is not None else self.default_temperature
        timeout_s = timeout_s or self.default_timeout
        max_retries = max_retries if max_retries is not None else self.default_retries

        # Ensure model is loaded
        if self._loaded_model != model:
            loaded = await self.load_model(model)
            if not loaded:
                return ("", False)

        # Build payload
        payload: Dict[str, Any] = {
            "model": model,
            "prompt": prompt,
            "stream": False,
            "options": {
                "temperature": temperature,
                "num_predict": 1024,
            },
        }
        if system_prompt:
            payload["system"] = system_prompt
        if images:
            # Verify model supports vision
            model_info = SUPPORTED_MODELS.get(model, {})
            if model_info.get("supports_vision", False):
                payload["images"] = images
            else:
                logger.warning(
                    f"Model '{model}' does not support vision. "
                    f"Ignoring images.")
        if response_format == "json":
            payload["format"] = "json"

        # Retry loop
        last_error = ""
        for attempt in range(max_retries + 1):
            try:
                session = await self._get_session()
                t_start = time.monotonic()

                async with session.post(
                    f"{self.base_url}/api/generate",
                    json=payload,
                    timeout=aiohttp.ClientTimeout(total=timeout_s),
                ) as resp:
                    elapsed_ms = (time.monotonic() - t_start) * 1000

                    if resp.status != 200:
                        body = await resp.text()
                        last_error = f"HTTP {resp.status}: {body[:200]}"
                        logger.warning(
                            f"Generate attempt {attempt+1} failed: "
                            f"{last_error}")
                        continue

                    data = await resp.json()
                    response_text = data.get("response", "").strip()

                    logger.info(
                        f"LLM generated {len(response_text)} chars "
                        f"in {elapsed_ms:.0f}ms "
                        f"(model={model}, attempt={attempt+1})")

                    # JSON validation if requested
                    if response_format == "json":
                        valid, response_text = self._validate_json(
                            response_text)
                        if not valid and attempt < max_retries:
                            # Re-prompt with correction
                            payload["prompt"] = (
                                f"{prompt}\n\n"
                                f"IMPORTANT: Your previous response was "
                                f"not valid JSON. Respond ONLY with a "
                                f"valid JSON object. No markdown, no "
                                f"code fences, no explanation."
                            )
                            logger.warning(
                                "Invalid JSON response, retrying with "
                                "correction prompt")
                            continue
                        elif not valid:
                            return ("", False)

                    return (response_text, True)

            except asyncio.TimeoutError:
                last_error = f"Timeout after {timeout_s}s"
                logger.warning(
                    f"Generate attempt {attempt+1} timed out "
                    f"after {timeout_s}s")
            except aiohttp.ClientError as e:
                last_error = str(e)
                logger.warning(
                    f"Generate attempt {attempt+1} error: {e}")
            except Exception as e:
                last_error = str(e)
                logger.error(
                    f"Unexpected error in generate: {e}")
                break

        logger.error(
            f"All {max_retries+1} generate attempts failed. "
            f"Last error: {last_error}")
        return ("", False)

    async def _generate_gemini(
        self,
        prompt: str,
        model: Optional[str] = None,
        system_prompt: str = "",
        images: Optional[List[str]] = None,
        response_format: Literal["text", "json"] = "text",
        temperature: Optional[float] = None,
        timeout_s: Optional[float] = None,
        max_retries: Optional[int] = None,
    ) -> Tuple[str, bool]:
        """Generate via Google Gemini API."""
        import base64

        model = model or self.default_model
        temperature = temperature if temperature is not None else self.default_temperature
        timeout_s = timeout_s or self.default_timeout
        max_retries = max_retries if max_retries is not None else self.default_retries

        if not self._gemini_key:
            logger.error("Gemini API key not configured")
            return ("", False)

        # Gemini model name (flash is fast and cheap)
        gemini_model = "gemini-1.5-flash"

        # Build the request payload
        contents = [{"role": "user", "parts": [{"text": prompt}]}]

        # Add system prompt if provided
        if system_prompt:
            contents.insert(0, {"role": "user", "parts": [{"text": system_prompt}]})

        # Build URL with key
        url = f"{self._gemini_endpoint}?key={self._gemini_key}"

        # Payload
        payload = {
            "model": gemini_model,
            "contents": contents,
            "temperature": temperature,
            "max_output_tokens": 1024,
        }

        # Retry loop
        last_error = ""
        for attempt in range(max_retries + 1):
            try:
                async with aiohttp.ClientSession() as session:
                    async with session.post(
                        url,
                        json=payload,
                        timeout=aiohttp.ClientTimeout(total=timeout_s),
                    ) as resp:
                        if resp.status != 200:
                            body = await resp.text()
                            last_error = f"HTTP {resp.status}: {body[:200]}"
                            logger.warning(
                                f"Generate attempt {attempt+1} failed: "
                                f"{last_error}")
                            continue

                        data = await resp.json()
                        # Gemini response structure
                        if data.get("candidates") and data["candidates"][0].get("content"):
                            response_text = data["candidates"][0]["content"]["parts"][0]["text"].strip()
                        else:
                            last_error = "Unexpected Gemini response structure"
                            logger.warning(f"Generate attempt {attempt+1}: {last_error}")
                            continue

                        logger.info(
                            f"Gemini generated {len(response_text)} chars "
                            f"in attempt {attempt+1}")

                        # JSON validation if requested
                        if response_format == "json":
                            valid, response_text = self._validate_json(
                                response_text)
                            if not valid and attempt < max_retries:
                                # Re-prompt with correction
                                # For Gemini, we just retry with the same prompt
                                logger.warning(
                                    "Invalid JSON response, retrying")
                                continue
                            elif not valid:
                                return ("", False)

                        return (response_text, True)

            except asyncio.TimeoutError:
                last_error = f"Timeout after {timeout_s}s"
                logger.warning(
                    f"Gemini generate attempt {attempt+1} timed out "
                    f"after {timeout_s}s")
            except aiohttp.ClientError as e:
                last_error = str(e)
                logger.warning(
                    f"Gemini generate attempt {attempt+1} error: {e}")
            except Exception as e:
                last_error = str(e)
                logger.error(f"Unexpected Gemini error: {e}")
                break

        logger.error(
            f"All {max_retries+1} Gemini generate attempts failed. "
            f"Last error: {last_error}")
        return ("", False)

    # ── Chat-style generation (for multi-turn) ─────────────
    async def chat(
        self,
        messages: List[Dict[str, str]],
        model: Optional[str] = None,
        response_format: Literal["text", "json"] = "text",
        temperature: Optional[float] = None,
        timeout_s: Optional[float] = None,
        max_retries: Optional[int] = None,
    ) -> Tuple[str, bool]:
        """
        Chat-style generation for multi-turn conversations.

        Args:
            messages: List of {"role": "system"|"user"|"assistant",
                       "content": "..."}
            model:    Model name
            response_format: "text" or "json"

        Returns:
            (response_text, success)
        """
        model = model or self.default_model
        temperature = temperature if temperature is not None else self.default_temperature
        timeout_s = timeout_s or self.default_timeout
        max_retries = max_retries if max_retries is not None else self.default_retries

        if self.mode == 'gemini':
            # Convert messages to Gemini format
            gemini_contents = []
            system_text = None
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                if role == "system":
                    system_text = content
                else:
                    gemini_contents.append({"role": role, "parts": [{"text": content}]})

            # Build payload
            gemini_payload = {
                "model": "gemini-1.5-flash",
                "contents": gemini_contents,
                "temperature": temperature,
                "max_output_tokens": 1024,
            }

            url = f"{self._gemini_endpoint}?key={self._gemini_key}"

            for attempt in range(max_retries + 1):
                try:
                    async with aiohttp.ClientSession() as session:
                        async with session.post(
                            url,
                            json=gemini_payload,
                            timeout=aiohttp.ClientTimeout(total=timeout_s),
                        ) as resp:
                            if resp.status != 200:
                                continue
                            data = await resp.json()
                            if data.get("candidates") and data["candidates"][0].get("content"):
                                content = data["candidates"][0]["content"]["parts"][0]["text"].strip()
                                return (content, True)
                            continue
                except (asyncio.TimeoutError, aiohttp.ClientError):
                    continue

            return ("", False)
        else:
            # Local Ollama chat
            # Convert messages to Ollama format
            formatted_prompt = ""
            for msg in messages:
                role = msg.get("role", "user")
                content = msg.get("content", "")
                if role == "system":
                    formatted_prompt += f"<system>{content}</system>\n"
                elif role == "user":
                    formatted_prompt += f"<user>{content}</user>\n"
                elif role == "assistant":
                    formatted_prompt += f"<assistant>{content}</assistant>\n"

            return await self._generate_ollama(
                formatted_prompt, model=model,
                response_format=response_format,
                temperature=temperature,
                timeout_s=timeout_s,
                max_retries=max_retries)

    # ── JSON validation ────────────────────────────────────
    @staticmethod
    def _validate_json(text: str) -> Tuple[bool, str]:
        """
        Validate and clean JSON response.
        Strips markdown code fences if present.

        Returns (is_valid, cleaned_text).
        """
        cleaned = text.strip()

        # Strip markdown code fences
        if cleaned.startswith("```"):
            lines = cleaned.split('\n')
            # Remove first line (```json) and last line (```)
            if lines[-1].strip() == '```':
                lines = lines[1:-1]
            else:
                lines = lines[1:]
            cleaned = '\n'.join(lines).strip()

        try:
            parsed = json.loads(cleaned)
            # Re-serialize to ensure clean formatting
            return (True, json.dumps(parsed, indent=2))
        except json.JSONDecodeError:
            return (False, cleaned)

    # ── Properties ─────────────────────────────────────────
    @property
    def loaded_model(self) -> Optional[str]:
        """Currently loaded model name, or None."""
        return self._loaded_model

    @property
    def is_model_loaded(self) -> bool:
        """Whether a model is currently loaded in VRAM."""
        return self._loaded_model is not None

    def get_model_info(self, model_name: str) -> Optional[Dict]:
        """Get model metadata from the registry."""
        return SUPPORTED_MODELS.get(model_name)

    def __repr__(self) -> str:
        return (
            f"OllamaClient(base_url={self.base_url!r}, mode={self.mode!r}, "
            f"loaded={self._loaded_model!r}, "
            f"gemini_key_set={bool(self._gemini_key)})")