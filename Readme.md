<div align="center">

# 🧠⚙️ AI ASSISTANT FOR INSTRUMENTATION & CONTROL
### Enterprise-Grade Closed-Loop Industrial Automation & Supervisory Framework

![Architecture](https://img.shields.io/badge/Architecture-Hybrid%20PID--LLM-blue?style=flat-square)
![Model](https://img.shields.io/badge/Model-Qwen3--4B%20(4--bit)-orange?style=flat-square)
![Protocol](https://img.shields.io/badge/Protocol-OPC%20UA-green?style=flat-square)
![Runtime](https://img.shields.io/badge/Runtime-CODESYS%20V3.5-purple?style=flat-square)
![HMI](https://img.shields.io/badge/HMI-Streamlit-red?style=flat-square)

</div>


## 📋 1. SYSTEM BLUEPRINT & ARCHITECTURE

This repository hosts an advanced **Digital Twin and AI Supervisory Control System** designed to bridge high-level generative intelligence with low-level deterministic Programmable Logic Controllers (PLCs)[cite: 66]. The system features a locally hosted, fine-tuned Qwen3-4B model acting as an active "AI Operator" that oversees continuous physical processes while a custom algorithmic layer handles real-time control physics[cite: 66].

### System Architecture Diagram

<p align="center">
  <img src="assets/architecture_diagram.png" width="850" alt="System Architecture Diagram">
</p>
---

## ⚙️ 2. ENGINEERING MODULES & CORE COMPONENTS

| Module ID | Component Name | Underlying Technology | Operational Role & Purpose |
| :--- | :--- | :--- | :--- |
| **MOD-01** | **Industrial Simulation** | CODESYS V3.5 SoftPLC | Emulates physical plant dynamics (Tank Level, Boiler Temp, Pressure) with strict safety clamping[cite: 67]. |
| **MOD-02** | **Communication Bridge** | Asynchronous OPC UA (`asyncua`) | Establishes a non-blocking 1.0s cyclic data stream over `opc.tcp://localhost:4840`[cite: 67]. |
| **MOD-03** | **Algorithmic Control** | Custom Deadband-Compensating PID | Overcomes 45%–55% actuator stiction via heuristic jumps (`+55.2` / `-44.8`) and tolerance idling[cite: 67]. |
| **MOD-04** | **Intelligence Engine** | Qwen3-4B (LoRA, 4-bit Quantized) | Executes local on-premise inference, dynamic context injection, and regex command generation. |
| **MOD-05** | **Supervisory HMI** | Streamlit Web Framework | Renders live telemetry, process gauges, system state badges, and deduplicated alarms[cite: 68]. |

---

## 🚀 3. EXECUTION & DEPLOYMENT WORKFLOW

### Prerequisites & Hardware Specifications
* **GPU:** NVIDIA GTX 1650 (4 GB GDDR6 VRAM)
* **CPU:** Intel Core i5 / AMD Ryzen 5 or higher[cite: 20]
* **System RAM:** 16 GB minimum
* **Software:** Python 3.10+[cite: 37], CODESYS V3.5 (Control Win SysTray x64), LM Studio[cite: 28], Anaconda.

1. **Initialize SoftPLC Runtime (CODESYS):**
   * Open **CODESYS V3.5** and load the project solution.
   * Ensure **CODESYS Control Win SysTray x64** is active in your taskbar tray.
   * Log in using administrative credentials (`Username: admin` / `Password: Admin1234!`).
   * Navigate to the **Debug** menu and click **Start** to broadcast tags on `opc.tcp://localhost:4840`.

2. **Spin Up Local Intelligence Engine (LM Studio):**
   * Open **LM Studio** and load the fine-tuned 4-bit quantized **Qwen3-4B** model weights into local GPU VRAM.
   * Start the local server endpoint at port `1234`.

3. **Launch the Orchestrator & Dashboard:**
   * Open an **Anaconda Prompt** terminal with CUDA configuration.
   * Execute the Streamlit deployment command:
     ```bash
     python -m streamlit run src/dashboard_app.py
     ```
   * Access the HMI web interface in your browser at `http://localhost:8501`.

## 📁 4. REPOSITORY DIRECTORY STRUCTURE

```text
├── assets/
│   └── architecture_diagram.png    # System architecture diagram
├── .gitignore                      # Git exclusion rules
├── Finetuning_Main.ipynb          # Model training & fine-tuning notebook
├── Finetuning_data.json            # Domain-specific training dataset
├── PLC_PRG_Version2.projectarchive # CODESYS V3.5 SoftPLC project archive
├── System Architecture Diagram.jpg # Raw system diagram asset
├── dashboard_app.py                # Main Streamlit HMI & orchestrator application
├── requirements.txt                # Python package dependencies
└── Readme.md                       # Repository documentation
