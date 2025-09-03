# 🧲 Dynamic Modelling & Control of a Magnetic Levitation System – MATLAB & Simulink  

## 📖 Introduction  
This project demonstrates the **modeling, analysis, and control** of a **magnetic levitation (maglev) system**, where a **ferromagnetic ball** is suspended under an **electromagnet**.  

The goal is to maintain the ball’s position at a **desired reference point** by regulating the input voltage to the coil.  
Due to the system’s **nonlinear dynamics** and **open-loop instability**, it provides a rich case study for modern control system design.  

The project includes:  
- **Nonlinear state-space modeling** and **linearization**  
- **Stability, controllability, and observability analysis**  
- **Transfer function derivation**  
- **PID controller design & tuning**  
- **Nonlinear simulations in MATLAB/Simulink** with **3D visualization**  

---

## 🔬 Methodology  
### 1. **System Modeling**  
- Derived a **nonlinear state-space model** based on Newton’s laws and electrical circuit dynamics.  
- Considered system states: **ball position, velocity, and coil current**.  
- Linearized the system around equilibrium points for analysis.  

### 2. **Control Design**  
- Implemented **P**, **PD**, and **PID** controllers for stabilization.  
- Tuned parameters to minimize overshoot, settling time, and steady-state error.  
- Verified stability using **step response analysis** and eigenvalue evaluation.  

### 3. **Simulation**  
- Built a **nonlinear Simulink model** of the maglev system.  
- Implemented **PID control** using MATLAB’s built-in blocks.  
- Simulated ball levitation with **3D visualization of motion**.  

---

## 🗂️ Project Structure  
<br />
├── 📄 2022MC45.prj                  # MATLAB project file  
<br />
├── 📄 maglev_nonlinear.slx          # Nonlinear Simulink model  
<br />
├── 📄 maglev.wrl                    # 3D visualization model  
<br />
├── 📄 maglev.x3d                    # 3D simulation data  
<br />
├── 📄 AppendixA.m                   # MATLAB code for analysis  
<br />
├── 📄 CEA_Report_2022_MC_45.pdf     # Full project report  
<br />
├── 📄 ReadMe.txt                    # Extra notes  
<br />
└── 📄 README.md                     # Project documentation  

---

📌 **Key Insights**  
- Demonstrates how **nonlinear systems** can be modeled and controlled using classical and modern techniques.  
- Shows the importance of **PID tuning** in stabilizing unstable systems.  
- Highlights the use of **MATLAB & Simulink** for real-world control engineering problems.  
