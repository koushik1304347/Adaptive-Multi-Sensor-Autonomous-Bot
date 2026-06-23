# Adaptive Multi-Sensor Autonomous Bot

A robust multi-agent Unmanned Ground Vehicle (UGV) simulation built in Webots. This project demonstrates autonomous navigation, dynamic obstacle avoidance, and Vehicle-to-Vehicle (V2V) communication using multiple sensor modalities.

The system is designed to allow multiple UGVs to navigate a shared environment, adapt to static physical obstacles, and yield to one another seamlessly without central network dependencies.

## Key Features

* **Graph-Based Path Planning:** Utilizes Dijkstra's algorithm to navigate a predefined grid of waypoints.
* **V2V Global Mapping:** Robots use simulated radio (Emitter/Receiver) to broadcast discovered obstacles. If one UGV detects a blocked road, all other agents update their global maps instantly.
* **Dynamic Rerouting:** UGVs continuously evaluate their current path and will dynamically replan in real-time if a newly broadcasted obstacle compromises their route.
* **Car-to-Car Collision Avoidance:** Includes GPS-based proximity tracking. When UGVs get too close, a right-of-way yielding logic activates to prevent deadlocks and crashes.
* **Adaptive Sensor Processing:** Integrates local depth camera data for physical obstacle detection, alongside predictive machine learning elements to determine optimal camera suitability under varying conditions (note: the ML model predicts sensor suitability, it does not directly drive the chassis).

##  Prerequisites

* [Webots](https://cyberbotics.com/) (Version R2025a or compatible)
* Python 3.x (Ensure Python is configured as the default controller language in Webots)

## Installation & Setup

Running the simulation is straightforward. The project is contained entirely within the `ugv_sim` folder.

1. Clone or download this repository to your local machine.
2. Copy the `ugv_sim` folder.
3. Paste the folder directly into your Webots project workspace (e.g., inside the Webots `worlds` directory, or wherever you organize your Webots projects).

##  How to Run

1. Launch the Webots application.
2. Go to **File > Open World...** and navigate to the pasted `ugv_sim/worlds/` directory.
3. Open the main world file (e.g., `grass_updated.wbt`).
4. Hit the **Play** button at the top of the Webots interface.

You can monitor the UGVs' decision-making processes—including path generation, obstacle detection, radio broadcasts, and yielding behaviors—directly in the Webots console.

##  Repository Structure

```text
Adaptive-Multi-Sensor-Autonomous-Bot/
├── ugv_sim/
│   ├── controllers/
│   │   └── controller_cl/
│   │       ├── controller_cl.py    # Main UGV state machine and driving logic
│   │       └── dijkstra_nav.py     # Graph data and pathfinding algorithms
│   └── worlds/
│       └── grass_updated.wbt       # The Webots environment and UGV nodes
└── README.md

```
