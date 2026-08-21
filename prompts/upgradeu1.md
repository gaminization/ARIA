<context>
This is UPGRADE U1 for ARIA — the LLM Planning and Reasoning upgrade.

Prerequisites: All 4 main ARIA stages complete and validated.
  ✅ Stage 1: Arm in Gazebo, manual control working
  ✅ Stage 2: IK, perception, grasp pipeline working
  ✅ Stage 3: All 15 agents, planning, dashboard working
  ✅ Stage 4: Real hardware connected, bidirectional servo sync working

CURRENT STATE (what exists):
  PlanningAgent uses rule-based NLP with hardcoded command patterns:
    "pick up X"     → [locate_X, plan_grasp_X, execute_grasp_X]
    "put X in Y"    → [locate_X, locate_Y, ...pick_X, place_in_Y]
    etc.
  This works for known command structures but fails on:
    - Ambiguous instructions
    - Novel multi-step tasks not in the template library
    - References to past sessions ("do what we did yesterday")
    - Conditional logic ("if the box is full, use the other box")
    - Long-horizon reasoning ("organize by color then by size")

UPGRADE GOAL:
  Replace the rule-based PlanningAgent parser with a local LLM.
  Add function calling pattern for flexible agent orchestration.
  Add Tree-of-Thought for complex multi-step tasks.
  Add episodic memory for cross-session task recall.
  
  Critical constraint:
    LLM is NOT always online. Ollama may be down.
    Rule-based parser must remain as a fallback.
    NEVER remove the original planning_agent.py.
    All upgrades are additive and the system gracefully
    degrades to rule-based if LLM is unavailable.
    
  VRAM constraint — RTX 5060 has 8GB:
    Llama 3.1 8B Q4_K_M (Ollama quantized): ~5GB VRAM
    This leaves ~3GB for YOLO + depth models simultaneously.
    Qwen2.5-VL 7B Q4: ~5GB VRAM (multimodal, can take images)
    Do NOT load LLM and GraspNet simultaneously.
    LLM inference on planning (rare) vs model inference (continuous)
    → LLM loads when planning, unloads during execution.
    → Implement: lazy load + explicit unload after plan is generated.

Hardware: Lenovo LOQ — i7-13700HX, RTX 5060 8GB
Software: Pop!_OS 22.04, ROS2 Humble, ARIA full stack
</context>

<task>
Generate the complete LLM intelligence upgrade for ARIA.
All files production-ready. No stubs. No pseudocode.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 1 — OLLAMA SETUP AND LLM CLIENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/llm_client.py

class OllamaClient:
  """
  Thin async wrapper around Ollama local inference server.
  Handles: model loading, unloading, structured JSON output,
  function calling schema, timeout/retry, fallback detection.
  
  Ollama API: http://localhost:11434
  """
  
  SUPPORTED_MODELS = {
    "llama3.1:8b-instruct-q4_K_M": {
      "vram_gb": 5.0,
      "context_length": 128000,
      "supports_vision": False,
      "use_for": ["planning", "reasoning", "dialogue"]
    },
    "qwen2.5-vl:7b-instruct-q4_K_M": {
      "vram_gb": 5.2,
      "context_length": 32768,
      "supports_vision": True,
      "use_for": ["visual_planning", "scene_description"]
    },
    "llama3.1:8b-instruct-q8_0": {
      "vram_gb": 8.5,   # WARNING: leaves almost no VRAM for perception
      "context_length": 128000,
      "supports_vision": False,
      "use_for": ["planning_high_quality"]  # only when nothing else running
    }
  }
  
  async def load_model(model_name: str) -> bool:
    """
    Explicitly pre-load model into VRAM via Ollama /api/pull.
    Waits until model is ready.
    Checks available VRAM first — warns if tight.
    Returns True if loaded successfully.
    """
    
  async def unload_model(model_name: str) -> bool:
    """
    Explicitly unload model to free VRAM.
    Called after planning is complete and execution begins.
    Uses Ollama keep_alive=0 parameter to force unload.
    """
    
  async def generate(
    prompt: str,
    model: str = "llama3.1:8b-instruct-q4_K_M",
    system_prompt: str = "",
    images: Optional[List[str]] = None,  # base64, for Qwen VL
    response_format: Literal["text", "json"] = "text",
    temperature: float = 0.1,            # low temp for planning (deterministic)
    timeout_s: float = 30.0,
    max_retries: int = 2
  ) -> Tuple[str, bool]:
    """
    Returns (response_text, success).
    On failure: returns ("", False) — caller handles fallback.
    
    JSON mode: instructs model to respond ONLY with valid JSON.
    Validates JSON parse before returning. Re-prompts once if invalid.
    """
    
  async def is_available(self) -> bool:
    """
    Quick health check: GET http://localhost:11434/api/tags
    Returns True if Ollama server is running and responsive.
    Fast timeout: 2 seconds.
    """
    
  async def get_vram_usage(self) -> Dict[str, float]:
    """
    Returns current VRAM usage for each loaded model.
    Uses nvidia-smi via subprocess.
    """

File: arm_planner/config/llm_config.yaml

  preferred_model: "llama3.1:8b-instruct-q4_K_M"
  fallback_to_rulebased: true
  temperature: 0.1
  timeout_s: 30.0
  max_retries: 2
  
  # When to use vision model (Qwen) vs text model (Llama)
  use_vision_for_planning: false   # start false, enable when needed
  vision_model: "qwen2.5-vl:7b-instruct-q4_K_M"
  
  # VRAM management
  unload_after_planning: true      # free VRAM during execution
  vram_headroom_gb: 2.5            # minimum free VRAM before loading LLM
  
  # Confidence thresholds
  llm_plan_confidence_threshold: 0.75
  require_approval_below: 0.65

File: arm_planner/prompts/system_prompt_planner.txt

  Write the complete system prompt for the planning LLM.
  
  The prompt should define:
    Role: "You are ARIA's planning module..."
    
    Available agents (as tools):
      VisionAgent, DepthAgent, TrackingAgent, AffordanceAgent,
      ControlAgent, SafetyAgent, MemoryAgent, WorldModelAgent,
      SkillAgent, ReachabilityAgent
      
    Available skills (as callable actions):
      pick(object_id), place(object_id, target),
      push(object_id, direction, distance),
      pull(object_id, direction, distance),
      stack(object_id, base_id),
      sort(object_ids, zone_map),
      inspect(object_id),
      slide(object_id, target_xy),
      roll(object_id, target_xy),
      sweep(target_area)
      
    Output format (strict JSON):
      {
        "goal": "string description of overall goal",
        "subgoals": ["subgoal1", "subgoal2", ...],
        "actions": [
          {
            "step": 1,
            "agent": "agent_name",
            "skill": "skill_name",
            "parameters": {},
            "reasoning": "why this step",
            "preconditions": ["condition1"],
            "expected_outcome": "what should happen"
          }
        ],
        "confidence": 0.0-1.0,
        "ambiguity_notes": "any unclear parts of the command",
        "requires_approval": true/false,
        "alternative_interpretations": []  // if ambiguous
      }
      
    World context (injected at runtime):
      Current objects in world model
      Current joint state
      Last 5 completed tasks (episodic memory)
      
    Constraints to follow:
      Safety first, never plan motions beyond workspace
      Always check reachability before planning grasp
      If confidence < 0.75, set requires_approval=true
      
  This is the most important prompt in the system.
  Write it carefully. Include examples (few-shot).
  
File: arm_planner/prompts/system_prompt_dialogue.txt

  System prompt for DialogueAgent's LLM mode.
  Focused on: natural explanation, approval requests,
  failure descriptions in plain language.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 2 — LLM PLANNING AGENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/llm_planning_agent.py

class LLMPlanningAgent(LifecycleNode):
  """
  Replaces rule-based PlanningAgent with LLM-backed planning.
  Registers on the same service as PlanningAgent so TaskManager
  does not change. Drop-in upgrade.
  
  Service: /aria/planning/decompose (same as original)
    Request:  {command: str, world_context: WorldContextMsg}
    Response: {goal: str, subgoals: [], actions: [], confidence: float}
  
  Startup:
    1. Check if Ollama is available
    2. If yes: activate LLM mode
    3. If no:  activate fallback mode (import original PlanningAgent)
    4. Publish /aria/planning/mode = "llm" or "rulebased"
    
  Planning pipeline:
  
    Step 1: Build world context string
      Current objects: list from WorldModelAgent
      Semantic relations: "cup is left of bottle"
      Joint state: current arm position
      Task history: last 5 tasks from EpisodicMemory
      
    Step 2: Load LLM (if not loaded)
      Check VRAM headroom
      Call client.load_model()
      
    Step 3: Construct prompt
      system_prompt_planner.txt + runtime context
      
    Step 4: Generate plan
      client.generate(prompt, response_format="json")
      
    Step 5: Validate plan
      JSON schema validation
      Check all referenced objects exist in world model
      Check all skills are registered in SkillManager
      Check confidence threshold
      
    Step 6: Unload LLM
      client.unload_model() → free VRAM for execution
      
    Step 7: Return plan to TaskManager
    
  Fallback handling (every step can fail independently):
    If Ollama unavailable → fallback to rule-based
    If JSON parse fails (after retry) → fallback
    If confidence < 0.4 → reject + ask user to rephrase
    If plan references unknown object → ask clarifying question
    
  Publish:
    /aria/planning/llm_plan (String: raw JSON for logging)
    /aria/planning/mode (String: "llm" or "rulebased")
    /aria/planning/last_plan_time_ms (Float64)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 3 — FUNCTION CALLING ORCHESTRATOR
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/function_calling_orchestrator.py

class FunctionCallingOrchestrator:
  """
  Instead of TaskManager having hardcoded agent calls,
  the LLM dynamically selects which agent/tool to call
  and with what parameters.
  
  Pattern: OpenAI-style tool use, running locally via Ollama.
  The LLM generates a sequence of tool calls. The orchestrator
  executes them one by one and feeds results back to the LLM
  for the next decision.
  
  This is a ReAct (Reasoning + Acting) loop:
    LLM: "I need to find the bottle. I'll call VisionAgent.detect."
    Execute: VisionAgent.detect(class="bottle")
    Result: {id: 42, position: (0.23, 0.15, 0.05), confidence: 0.94}
    LLM: "Bottle found. Now I'll check if it's reachable."
    Execute: ReachabilityAgent.check(pose=...)
    Result: {reachable: true}
    LLM: "Reachable. Now I'll plan the grasp."
    ...
  """
  
  TOOL_SCHEMAS = {
    "detect_object": {
      "description": "Detect an object in the workspace",
      "parameters": {
        "class_name": "string — object class to find",
        "color": "optional string — filter by color"
      },
      "returns": "ObjectDetection with position and confidence"
    },
    "check_reachability": {
      "description": "Check if a pose is within the arm's workspace",
      "parameters": {
        "position": "[x, y, z] in meters",
        "orientation": "optional [rx, ry, rz, rw] quaternion"
      },
      "returns": "bool reachable, optional alternative_pose"
    },
    "get_grasp_strategy": {
      "description": "Get the best grasp pose for an object",
      "parameters": {
        "object_id": "int",
        "strategy_hint": "optional: top_down, side, handle"
      },
      "returns": "grasp_pose, approach_pose, confidence"
    },
    "execute_skill": {
      "description": "Execute a predefined robot skill",
      "parameters": {
        "skill_name": "one of: pick, place, push, pull, stack, sort, inspect",
        "skill_args": "dict of skill-specific arguments"
      },
      "returns": "success bool, failure_reason if failed"
    },
    "query_world_model": {
      "description": "Query the world model for object information",
      "parameters": {
        "query": "string — natural language query about the scene"
      },
      "returns": "WorldObject list matching query"
    },
    "ask_user": {
      "description": "Ask the user a clarifying question or approval",
      "parameters": {
        "question": "string",
        "question_type": "one of: approval, clarification, confirmation"
      },
      "returns": "user response string"
    }
  }
  
  async def run_react_loop(
    command: str,
    max_iterations: int = 15,
    timeout_s: float = 120.0
  ) -> TaskResult:
    """
    Full ReAct loop.
    Each iteration: LLM decides next tool → execute → feed result back.
    Terminates when LLM outputs {"action": "task_complete"} or
    {"action": "task_failed", "reason": "..."}.
    
    Detailed logging at each step (chain of thought format).
    All intermediate results stored in TaskState.
    """

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 4 — TREE-OF-THOUGHT PLANNER
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/tree_of_thought_planner.py

class TreeOfThoughtPlanner:
  """
  For complex multi-step tasks, generate multiple plan candidates,
  evaluate each, select the best.
  
  Only activated when:
    - Task has 5+ steps (simple tasks use standard LLM planning)
    - Task has conditional logic ("if X then Y else Z")
    - Task failed once and is being retried with different approach
    - User explicitly requests: "think carefully before acting"
    
  Tree structure:
    Root: initial task description
    Level 1: 3 high-level approaches (breadth=3)
    Level 2: expand each approach into detailed steps (depth=3)
    Evaluation: score each complete plan
    Selection: pick highest scoring plan
    
  VRAM note: ToT makes 3+ LLM calls. Load model once, keep loaded
  for all ToT calls, then unload. Add "thinker" mode flag.
  """
  
  def generate_plan_candidates(
    task: str,
    world_context: str,
    n_candidates: int = 3
  ) -> List[PlanCandidate]:
    """
    Prompt: "Generate {n} different approaches to: {task}
             Consider: {world_context}
             For each approach, give: approach_name, high_level_steps,
             estimated_success_probability, main_risk"
             
    Returns list of PlanCandidate objects.
    """
    
  def score_plan(plan: PlanCandidate, world_context: str) -> float:
    """
    Score each candidate plan on:
      feasibility_score:    all steps reachable + objects exist (0-1)
      efficiency_score:     fewer steps preferred (0-1)
      risk_score:           inverse of main_risk severity (0-1)
      precondition_score:   all preconditions checkable (0-1)
      
    total_score = 0.4*feasibility + 0.2*efficiency + 
                  0.3*risk + 0.1*precondition
                  
    Also runs ReachabilityAgent on key poses as pre-check.
    """
    
  def select_best_plan(candidates: List[PlanCandidate]) -> PlanCandidate:
    """Select highest scoring. Log all scores for transparency."""
    
  Publishes to chain_of_thought:
    "[ToT] Generated 3 plan candidates for: {task}"
    "[ToT] Plan A: score=0.87, approach=direct_grasp_then_place"
    "[ToT] Plan B: score=0.72, approach=clear_obstacles_first"
    "[ToT] Plan C: score=0.65, approach=push_then_grasp"
    "[ToT] Selected Plan A (highest score)"

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 5 — EPISODIC TASK MEMORY
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/arm_planner/episodic_memory.py

class EpisodicMemory:
  """
  Stores complete task episodes for cross-session recall.
  Enables: "do what we did yesterday", "repeat last task",
           "do the same but with the blue cube"
           
  Each episode stores:
    - Original natural language command
    - Generated plan (JSON)
    - Objects involved (class, color, size)
    - Outcome (success/fail)
    - Duration
    - Key world state snapshots
    - Timestamp
    
  Uses SQLite for persistence + vector embeddings for
  semantic search ("find tasks similar to this one").
  """
  
  Storage:
    Database: arm_planner/data/episodes.db
    
    Schema:
      CREATE TABLE episodes (
        id INTEGER PRIMARY KEY,
        command TEXT,
        goal TEXT,
        plan_json TEXT,
        objects_involved TEXT,  -- JSON list
        success INTEGER,
        duration_s REAL,
        timestamp TEXT,
        session_id TEXT,
        command_embedding BLOB  -- numpy float32 array
      )
      
  Embedding generation:
    Use sentence-transformers (all-MiniLM-L6-v2, tiny, CPU-only):
    pip install sentence-transformers
    model = SentenceTransformer('all-MiniLM-L6-v2')
    embedding = model.encode(command)  # 384-dim float32
    
    This runs on CPU, no VRAM used.
    Stored as BLOB in SQLite.
    
  Semantic search:
    def find_similar_tasks(query: str, top_k: int = 3) -> List[Episode]:
      query_embedding = model.encode(query)
      # Cosine similarity with all stored embeddings
      # Return top_k most similar episodes
      # Used by LLM: inject similar past tasks into planning context
      
  Natural language references:
    def resolve_temporal_reference(ref: str) -> Optional[Episode]:
      # "yesterday's task"  → episodes from yesterday, sorted by time
      # "last task"         → most recent episode
      # "what we did before"→ semantic search + recency weighting
      # "the sorting task"  → semantic search for "sort" tasks
      
  Context injection for LLM:
    def get_planning_context(command: str) -> str:
      similar = find_similar_tasks(command, top_k=3)
      # Format as: "Relevant past tasks:
      #   1. '{command}' → {outcome} ({n} days ago)
      #      Key steps: {plan_summary}"
      # Inject into system prompt at planning time

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 6 — UPGRADED DIALOGUE AGENT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_agents/arm_agents/llm_dialogue_agent.py

  Drop-in replacement for original DialogueAgent.
  Uses LLM for richer natural language responses.
  
  Capabilities added:
  
    Conversational task clarification:
      "Did you mean the red bottle on the left or the one in the box?"
      (LLM generates question based on detected ambiguity)
      
    Context-aware explanations:
      Instead of template: "Task failed: MISSED_OBJECT"
      LLM generates: "I tried to pick up the bottle but my gripper
                      closed about 2cm too far to the right. This
                      might be because the lighting made it hard to
                      see the exact position. Should I try again?"
                      
    Cross-session references:
      User: "Can you do what you did with the cups last time?"
      Agent: Queries EpisodicMemory → finds cup-sorting task
             → confirms with user → re-executes
             
    Proactive suggestions:
      If task succeeds consistently: "I've picked up cylinders
      64 times this week. Would you like me to add this as a
      named skill called 'get_cylinder'?"
      
  Visual planning (when Qwen2.5-VL enabled):
    User points phone at workspace
    Sends image to DialogueAgent
    Qwen describes scene + suggests task
    "I can see a red cube, a blue cylinder, and a white box.
     Would you like me to sort them by color?"

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
PART 7 — INTEGRATION + MIGRATION SCRIPT
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
File: arm_planner/scripts/upgrade_u1_install.sh

  One-time setup script:
  
  # Install Ollama
  curl -fsSL https://ollama.ai/install.sh | sh
  
  # Pull models (run overnight — llama3.1 8B is ~5GB)
  ollama pull llama3.1:8b-instruct-q4_K_M
  ollama pull qwen2.5-vl:7b-instruct-q4_K_M
  
  # Install Python dependencies
  conda activate aria
  pip install sentence-transformers ollama aiohttp
  
  # Verify Ollama running
  curl http://localhost:11434/api/tags
  
  # Test LLM client
  python -c "
  import asyncio
  from arm_planner.llm_client import OllamaClient
  client = OllamaClient()
  async def test():
      avail = await client.is_available()
      print('Ollama available:', avail)
  asyncio.run(test())
  "

File: arm_bringup/scripts/validate_upgrade_u1.py

  Tests:
  
  1. Ollama health check
     → client.is_available() returns True
     
  2. Model loads without OOM
     → Load llama3.1 8B, check VRAM < 6GB
     → nvidia-smi confirms model in VRAM
     
  3. Basic plan generation
     Command: "Pick up the red cube"
     → LLM generates valid JSON plan
     → JSON schema validates
     → Plan has at least 3 steps
     
  4. Complex plan generation
     Command: "Sort all the objects by color and stack the
              same-color ones together"
     → LLM generates plan with 8+ steps
     → Plan covers: detect all, group by color, pick/stack each
     
  5. Temporal reference
     (Requires at least 1 episode in memory)
     Command: "Do the last task again"
     → EpisodicMemory finds last episode
     → Plan matches original structure
     
  6. Ambiguity detection
     Command: "Pick up the thing"  (intentionally vague)
     → Confidence < 0.65
     → requires_approval = true
     → DialogueAgent asks clarifying question
     
  7. VRAM cleanup
     → After planning: model unloaded
     → nvidia-smi confirms VRAM freed
     
  8. Fallback to rule-based
     → Stop Ollama server
     → Send command
     → System uses rule-based fallback
     → /aria/planning/mode = "rulebased"
     → Restart Ollama → mode returns to "llm"
     
  Output:
  ══════════════════════════════════════════
  ARIA Upgrade U1 — LLM Planning Validation
  ══════════════════════════════════════════
  ✅ Ollama running
  ✅ Model loads (5.1GB VRAM)
  ✅ Simple plan: valid JSON, 4 steps
  ✅ Complex plan: valid JSON, 11 steps
  ✅ Temporal reference resolved
  ✅ Ambiguity detected, approval requested
  ✅ VRAM freed after planning
  ✅ Fallback to rule-based: working
  ══════════════════════════════════════════
  U1 UPGRADE COMPLETE
  ══════════════════════════════════════════
</task>

<output_order>
═══ FILE: path/to/file ═══

1.  arm_planner/arm_planner/llm_client.py
2.  arm_planner/config/llm_config.yaml
3.  arm_planner/prompts/system_prompt_planner.txt
4.  arm_planner/prompts/system_prompt_dialogue.txt
5.  arm_planner/arm_planner/llm_planning_agent.py
6.  arm_planner/arm_planner/function_calling_orchestrator.py
7.  arm_planner/arm_planner/tree_of_thought_planner.py
8.  arm_planner/arm_planner/episodic_memory.py
9.  arm_agents/arm_agents/llm_dialogue_agent.py
10. arm_planner/scripts/upgrade_u1_install.sh
11. arm_bringup/scripts/validate_upgrade_u1.py
12. arm_planner/arm_planner/llm_planning_agent_tests.py
13. arm_bringup/launch/aria_full_u1.launch.py
    (adds llm_planning_agent + llm_dialogue_agent to aria_full.launch.py)
14. README_UPGRADE_U1.md
    (setup guide, Ollama commands, VRAM guide, troubleshooting)

[U1 CHECKPOINT] Files: X/14 — Resume: <next_file>
</output_order>