# Grid-Connected STATCOM: dq0 Control

![STATCOM System Architecture](Docs/LaTeX_Source/Images/Full_Sys.svg)

![MATLAB/Simulink](https://img.shields.io/badge/MATLAB-Simulink-blue?logo=mathworks)
![Domain](https://img.shields.io/badge/Domain-Power_Quality_%2F_FACTS-red)

## 🚀 Overview
MATLAB/Simulink model of a grid-connected **Static Synchronous Compensator (STATCOM)** built on a **two-level Voltage Source Converter (VSC)**. It supplies the reactive power of a local inductive load so the grid sees a unity power factor.

Control uses the **synchronous reference frame (dq0)** with SPWM, giving decoupled regulation of the DC-link voltage and the reactive power.

## 🎯 Problem Statement
* **Grid:** 415 V (line-to-line RMS), 50 Hz.
* **Local load:** 200 kW active and 100 kVAR inductive reactive power.
* **Target:** the STATCOM injects 100 kVAR (capacitive) at the PCC so the grid supplies only active power (Q<sub>grid</sub> ≈ 0).
* **DC link:** 800 V across a 10 mF capacitor.

## 🧠 Control Strategy
1. **PLL** extracts the grid angle for the abc ↔ dq0 transformations.
2. **Cascaded PI loops**
   * *Outer loop:* regulates the DC-link voltage to 800 V and outputs the active-current reference I<sub>d</sub>* (covers converter losses).
   * *Inner loops:* regulate the d- and q-axis currents; I<sub>q</sub>* is set to compensate the 100 kVAR demand.
3. **SPWM:** the reference signals are compared with a **10 kHz** triangular carrier to drive the six IGBTs.

## 🔧 System Parameters
| Parameter | Value |
| :--- | :--- |
| Grid | 415 V (L-L RMS), 50 Hz |
| Load | 200 kW + 100 kVAR (inductive) |
| DC-link voltage / capacitor | 800 V / 10 mF |
| Switching frequency | 10 kHz |
| LCL filter | 300 µH and 500 µH inductors, 100 µF capacitors |

## ⚙️ Simulation Results
* **Reactive power:** at steady state (about 0.2 s) the STATCOM injects 100 kVAR and the grid reactive power goes to zero (unity power factor).
* **Active power:** the grid supplies the 200 kW load plus a small amount for the converter's switching losses.
* **DC link:** the outer loop charges the capacitor and holds 800 V.
* **Filtering:** the LCL filter attenuates the 10 kHz switching components in the grid current.

| Active Power Flow (P) | Reactive Power Flow (Q) |
| :---: | :---: |
| ![P Scope](Docs/LaTeX_Source/Images/P_Scope1.jpg) | ![Q Scope](Docs/LaTeX_Source/Images/Q_Scope3.jpg) |

## 📂 Repository Structure
* `Simulation/STATCOM.slx` — Simulink model with subsystems for control loop, conversion, gate control, LCL filter and measurement.
* `Docs/statcom-dq0-control.pdf` — report with theory and waveform analysis.
* `Docs/LaTeX_Source/` — LaTeX source and figures.

## ▶️ How to Run
Open `Simulation/STATCOM.slx` in MATLAB/Simulink (Simscape Electrical required) and press **Run**.

## 👨‍💻 Author
**Abd El-Rhman Muhammad Saad** — Electrical Power and Machines Engineering, Alexandria University.
[LinkedIn](https://linkedin.com/in/Abd-El-Rhman-Saad) · [GitHub](https://github.com/Abd-El-Rhman-Saad)
