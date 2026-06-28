#!/usr/bin/env python3
"""
═══════════════════════════════════════════════════════════════
ARIA LLM Client — Async Ollama Wrapper
Handles: model loading/unloading, structured JSON output,
function calling schema, timeout/retry, fallback detection.

Ollama API: http://localhost:11434

VRAM Management:
  RTX 5060 has 8GB. Llama 3.1 8B Q4_K_M uses ~5GB.
  LLM loads when planning, unloads during execution to
  free VRAM for YOLO + depth models.
═══════════════════════════════════════════════════════════════
"""
import asyncio
import json
import logging
import os
import subprocess
import time
from typing import Any, Dict, List, Literal, Optional, Tuple

import aiohttp
import yaml

logger = logging.getLogger(__name__)

# ═══════════════════════════════════════════════════════════════
# Configuration
# ═══════════════════════════════════════════════════════════════
_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'config', 'llm_config.yaml')


def _load_config() -> dict:
    """Load LLM configuration from YAML file."""
    try:
        with open(_CONFIG_PATH, 'r') as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        logger.warning("llm_config.yaml not found, using defaults")
        return {}


# ═══════════════════════════════════════════════════════════════
# Model Registry
# ═══════════════════════════════════════════════════════════════
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
# ═══════════════════════════════════════════════════════════════
class OllamaClient:
    """
    Thin async wrapper around Ollama local inference server.

    Features:
      - Lazy model loading with VRAM headroom checks
      - Explicit unloading to free VRAM during execution
      - Structured JSON mode with validation and retry
      - Health checking with fast timeout
      - VRAM usage monitoring via nvidia-smi
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

        # Config values with defaults
        self.default_model = self.config.get(
            'preferred_model', 'llama3.1:8b-instruct-q4_K_M')
        self.default_temperature = self.config.get('temperature', 0.1)
        self.default_timeout = self.config.get('timeout_s', 30.0)
        self.default_retries = self.config.get('max_retries', 2)
        self.vram_headroom = self.config.get('vram_headroom_gb', 2.5)
        self.unload_after_planning = self.config.get(
            'unload_after_planning', True)

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
        Quick health check: GET /api/tags
        Returns True if Ollama server is running and responsive.
        Fast timeout: 2 seconds.
        """
        try:
            session = await self._get_session()
            async with session.get(
                f"{self.base_url}/api/tags",
                timeout=aiohttp.ClientTimeout(total=2.0),
            ) as resp:
                return resp.status == 200
        except (aiohttp.ClientError, asyncio.TimeoutError, OSError):
            return False

    async def list_local_models(self) -> List[str]:
        """List all models available locally in Ollama."""
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

    # ── VRAM Management ────────────────────────────────────
    async def get_vram_usage(self) -> Dict[str, float]:
        """
        Returns current VRAM usage info via nvidia-smi.

        Returns:
            {
                "total_gb": float,
                "used_gb": float,
                "free_gb": float,
            }
        """
        try:
            result = await asyncio.get_event_loop().run_in_executor(
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
        Check if there is enough VRAM headroom to load a model.
        Returns True if safe to proceed.
        """
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
            # Allow loading even if tight — Ollama will manage
            # But warn if free < model size
            return available >= (required * 0.7)

        logger.info(
            f"VRAM OK: {available:.1f}GB free, "
            f"model needs {required:.1f}GB")
        return True

    # ── Model Loading / Unloading ──────────────────────────
    async def load_model(self, model_name: str) -> bool:
        """
        Explicitly pre-load model into VRAM.

        Uses Ollama /api/generate with an empty prompt and
        keep_alive=-1 to load the model persistently.
        Checks available VRAM first.

        Returns True if loaded successfully.
        """
        async with self._load_lock:
            if self._loaded_model == model_name:
                logger.info(f"Model '{model_name}' already loaded")
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
        Explicitly unload model to free VRAM.
        Uses keep_alive=0 to force immediate unload.

        Called after planning is complete and execution begins.
        """
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
            images:          Base64-encoded images (for Qwen VL)
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
                "num_predict": 4096,
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

        if self._loaded_model != model:
            loaded = await self.load_model(model)
            if not loaded:
                return ("", False)

        payload: Dict[str, Any] = {
            "model": model,
            "messages": messages,
            "stream": False,
            "options": {
                "temperature": temperature,
                "num_predict": 4096,
            },
        }
        if response_format == "json":
            payload["format"] = "json"

        for attempt in range(max_retries + 1):
            try:
                session = await self._get_session()
                async with session.post(
                    f"{self.base_url}/api/chat",
                    json=payload,
                    timeout=aiohttp.ClientTimeout(total=timeout_s),
                ) as resp:
                    if resp.status != 200:
                        continue
                    data = await resp.json()
                    content = data.get("message", {}).get(
                        "content", "").strip()

                    if response_format == "json":
                        valid, content = self._validate_json(content)
                        if not valid and attempt < max_retries:
                            messages.append({
                                "role": "user",
                                "content": (
                                    "Your response was not valid JSON. "
                                    "Respond ONLY with a valid JSON object."
                                ),
                            })
                            continue
                        elif not valid:
                            return ("", False)

                    return (content, True)

            except (aiohttp.ClientError, asyncio.TimeoutError):
                continue

        return ("", False)

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
            f"OllamaClient(base_url={self.base_url!r}, "
            f"loaded={self._loaded_model!r})")
