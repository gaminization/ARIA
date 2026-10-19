# ARIA LLM Configuration
# Set GEMINI_API_KEY to enable Google Gemini offloading
# Leave empty to use local Ollama inference

# Switch mode:
#   ENABLE_GEMINI=false  (default) -- use local Ollama
#   ENABLE_GEMINI=true   -- use Gemini when GEMINI_API_KEY is set
#   ENABLE_GEMINI=auto   -- use Gemini if key available, else local Ollama

GEMINI_API_KEY=''
ENABLE_GEMINI='auto'

# Local Ollama configuration
OLLAMA_BASE_URL='http://localhost:11434'
OLLAMA_MODEL='llama3.1:8b-instruct-q4_K_M'

# Gemini configuration
GEMINI_ENDPOINT='https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent'

