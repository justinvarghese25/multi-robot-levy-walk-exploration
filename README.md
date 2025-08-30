# Multi-Robot Lévy Walk Exploration with Potential Fields

A MATLAB-based simulation of a multi-robot exploration system that combines a modified Lévy walk strategy with potential fields for collision avoidance, dynamic map merging, and object detection in unknown environments.

---

## 🚀 Features

* **Multi-Robot Exploration**: Cooperative navigation of a bounded 2D map.
* **Modified Lévy Walk**: Balances local and global exploration steps.
* **Potential Fields**: Maintains safe inter-robot distances and avoids obstacles.
* **Dynamic Map Merging**: Combines local occupancy maps into a global representation.
* **Object Detection**: Marks detected objects in the merged map.
* **Performance**: Achieves >90% map coverage in simulation.

---

## 📂 Repository Structure

```
.
├── main.m                       # Entry point script
├── initializeEnvironment.m      # Sets up environment, maps, and parameters
├── initializeRobots.m           # Initializes robot poses, controllers, and sensors
├── levyWalkWithPotentialField.m # Navigation logic
├── mergeRobotMaps.m             # Merges local maps into a global map
├── plotResults.m                # Visualization of results
└── README.md                    # Project documentation
```

---

## ⚙️ Requirements

* MATLAB **R2022b** or later
* **Robotics System Toolbox**

---

## ▶️ How to Run

1. Clone the repository:

   ```bash
   git clone https://github.com/your-username/multi-robot-levy-exploration.git
   ```
2. Open **MATLAB** and navigate to the project folder.
3. Run the main simulation script:

   ```matlab
   main
   ```

---

## 📊 Simulation Output

Running the simulation will display:

* Robot trajectories
* Merged occupancy map with detected objects
* Potential energy plot

---

## 🔍 Simulation Details

* **Number of Robots**: 4
* **Map Size**: 26 × 27 grid
* **Sensor Range**: 7 units
* **Safe Distance**: 1.5 units
* **Object Detection Radius**: 2 units

---

## 📈 Results

* Achieved **over 90% coverage** of the environment.
* Maintained **collision-free navigation** throughout the simulation.
* Generated a **merged occupancy map** with detected object locations.

![Simulation Results](images/results.png)

---

## 🛠️ Future Work

* Integration of advanced object detection and classification.
* Real-world testing with physical robots in dynamic environments.

---

## 👤 Author

**Justin Varghese John**
M.S. Robotics & Autonomous Systems (AI)
Arizona State University
