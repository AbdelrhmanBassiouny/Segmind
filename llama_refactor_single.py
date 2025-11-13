import os
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer, GenerationConfig, BitsAndBytesConfig

# -------------------------
# Paths
# -------------------------
FILE_TO_REFACTOR = "/home/nvasant/workspace/ros/src/Segmind/src/segmind/detectors/atomic_event_detectors.py"
CONTEXT_DIRS = [
    "/home/nvasant/workspace/ros/src/pycram",
    "/home/nvasant/workspace/ros/src/semantic_digital_twin"
]
OUTPUT_DIR = "/home/nvasant/workspace/ros/src/Segmind/refactored"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# -------------------------
# Load project context
# -------------------------
def load_context(folders, exclude_file):
    context_code = ""
    for folder in folders:
        for root, dirs, files in os.walk(folder):
            for file in files:
                if file.endswith(".py"):
                    full_path = os.path.join(root, file)
                    if full_path != exclude_file:
                        with open(full_path, "r", encoding="utf-8") as f:
                            context_code += f"# File: {full_path}\n"
                            context_code += f.read() + "\n\n"
    return context_code

context_code = load_context(CONTEXT_DIRS, FILE_TO_REFACTOR)

# -------------------------
# Load the file to refactor
# -------------------------
with open(FILE_TO_REFACTOR, "r", encoding="utf-8") as f:
    code_to_refactor = f.read()

# -------------------------
# Hugging Face token
# -------------------------
hf_token = os.environ.get("HUGGINGFACE_HUB_TOKEN")
if hf_token is None:
    raise ValueError("HUGGINGFACE_HUB_TOKEN environment variable not set.")

# -------------------------
# Model configuration (VRAM-efficient)
# -------------------------
MODEL_NAME = "meta-llama/Llama-2-7b-hf"
print("Loading LLaMA 2 7B model with low VRAM setup...")

tokenizer = AutoTokenizer.from_pretrained(MODEL_NAME, token=hf_token)

quant_config = BitsAndBytesConfig(
    load_in_8bit=True,
    llm_int8_enable_fp32_cpu_offload=True  # CPU offload
)

model = AutoModelForCausalLM.from_pretrained(
    MODEL_NAME,
    device_map="auto",          # automatic GPU/CPU placement
    quantization_config=quant_config,
    torch_dtype=torch.float16,  # FP16 for GPU layers
    token=hf_token
)
model.eval()

# -------------------------
# Chunked input for memory efficiency
# -------------------------
CHUNK_SIZE = 1500  # tokens per chunk to avoid OOM
MAX_NEW_TOKENS = 512  # tokens generated per chunk

def chunk_text(text, tokenizer, chunk_size):
    tokens = tokenizer.encode(text)
    for i in range(0, len(tokens), chunk_size):
        yield tokenizer.decode(tokens[i:i+chunk_size])

refactored_chunks = []

for chunk in chunk_text(code_to_refactor, tokenizer, CHUNK_SIZE):
    prompt = f"""
You are a Python code assistant. Refactor the following code to:
- Replace pycram usage with semantic_digital_twin framework
- Improve readability and modularity
- Maintain original functionality

Project context (do not modify these files):
{context_code}

Original code chunk to refactor:
{chunk}

Return only the refactored code.
"""
    inputs = tokenizer(prompt, return_tensors="pt", truncation=True, max_length=2048).to("cuda")
    generation_config = GenerationConfig(
        max_new_tokens=MAX_NEW_TOKENS,
        do_sample=False
    )
    with torch.no_grad():
        outputs = model.generate(**inputs, generation_config=generation_config)
    refactored_chunks.append(tokenizer.decode(outputs[0], skip_special_tokens=True))
    torch.cuda.empty_cache()  # free VRAM between chunks

# -------------------------
# Save final refactored code
# -------------------------
refactored_code = "\n\n".join(refactored_chunks)
output_file = os.path.join(OUTPUT_DIR, os.path.basename(FILE_TO_REFACTOR))
with open(output_file, "w", encoding="utf-8") as f:
    f.write(refactored_code)

print(f"Refactored code saved to: {output_file}")
