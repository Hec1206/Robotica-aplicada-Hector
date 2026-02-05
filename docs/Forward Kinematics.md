# Forward Kinematics 

This repository is used to document *Denavit–Hartenberg (DH) frame placement* and *homogeneous transformation matrices* for multiple robots.

## Introduction

The **Denavit–Hartenberg (D–H) Convention** is a standardized method used in robotics to model the kinematics of serial manipulators. Its main purpose is to provide a systematic way to assign coordinate frames to each link of a robotic mechanism and to describe the spatial relationship between consecutive links using a minimal set of parameters.

By defining four parameters for each joint—**link length (`aᵢ`)**, **link twist (`αᵢ`)**, **link offset (`dᵢ`)**, and **joint angle (`θᵢ`)**—the Denavit–Hartenberg Convention simplifies the process of deriving **homogeneous transformation matrices**. These matrices describe the position and orientation of one coordinate frame with respect to another and are fundamental for forward kinematics analysis.

The use of the Denavit–Hartenberg method reduces complexity, avoids ambiguity in frame assignment, and allows robotic systems to be modeled in a consistent and repeatable manner. For this reason, it is widely used in the analysis, design, and control of robotic manipulators.


## Objectives

### **General Objective**
To apply the **Denavit–Hartenberg (D–H) Convention** for the kinematic modeling of robotic manipulators, with the purpose of systematically defining coordinate frames and obtaining the corresponding **homogeneous Denavit–Hartenberg transformation matrices**.

### **Specific Objectives**
- **OE1:** To identify and correctly define the four Denavit–Hartenberg parameters  
  *(link length `aᵢ`, link twist `αᵢ`, link offset `dᵢ`, and joint angle `θᵢ`)* according to the standard D–H rules.
- **OE2:** To assign coordinate frames to robotic links following the **Denavit–Hartenberg Convention**, ensuring proper axis orientation and consistency between consecutive frames.
- **OE3:** To construct the **Denavit–Hartenberg parameter tables** for different robotic configurations while respecting the convention’s parameters and constraints.
- **OE4:** To solve **five kinematic exercises** using the Denavit–Hartenberg method, deriving the corresponding **D–H homogeneous transformation matrices** for each case.
- **OE5:** To analyze and validate the obtained transformation matrices by verifying their mathematical correctness and physical interpretation within robotic kinematics.

!!! note "Note"
    in ever excersie in the scheme The coordinate frames are assigned according to the **Denavit–Hartenberg Convention**, where each **z-axis** is aligned with the joint axis, and each **x-axis** is defined along the common normal between consecutive joints. This assignment allows the direct extraction of the D–H parameters and the formulation of the corresponding transformation matrices. and in the transformation Matrix i am using the assigned frames and the Denavit–Hartenberg parameters, the homogeneous transformation matrix represents the spatial relationship between consecutive links, allowing the kinematic modeling of the robot.

---

# Robot 1

## Robot Photo
![Robot 1 photo](imgs2/ejercicio1.png)

This figure shows the physical configuration of **Robot 1**, including its links and joints. 

---

## Frame Assignment / Scheme
![Robot 1 scheme](imgs2/ejercicio1.1.png)

## DH Transformation Matrix
![Robot 1 DH](imgs2/ejercicio1.3.png)

---

# Robot 2

## Robot Photo
![Robot 2 photo](imgs2/ejercicio2.png)

This figure shows the physical configuration of **Robot 2**, including its links and joints.

---

## Frame Assignment / Scheme
![Robot 2 scheme](imgs2/ejercico2.1.jpeg)

## DH Transformation Matrix
![Robot 2 DH](imgs2/ejercico2.2.jpeg) 
![](imgs2/ejercico2.3.jpeg)

---

# Robot 3

## Robot Photo
![Robot 3 photo](imgs2/ejercicio3.png)

This figure shows the physical configuration of **Robot 3**, including its links and joints. 

---

## Frame Assignment / Scheme
![Robot 3 scheme](imgs2/ejercicio3.1.png) 
![](imgs2/ejercicio3.2.png) 

## DH Transformation Matrix
![Robot 3 DH](imgs2/ejercico3.3.jpeg) 
![](imgs2/ejercico3.4.jpeg) 

---

# Robot 4

## Robot Photo
![Robot 4 photo](imgs2/ejercicio4.png) 

This figure shows the physical configuration of **Robot 4**, including its links and joints.

---

## Frame Assignment / Scheme
![Robot 4 scheme](imgs2/ejercicio4.1.png) 
![](imgs2/ejercicio4.2.png) 
![](imgs2/ejercicio4.3.png) 
![](imgs2/ejercicio4.4.png) 

## DH Transformation Matrix
![Robot 4 DH](imgs2/ejercicio4.2.png) 

---

# Robot 5

## Robot Photo
![Robot 5 photo](imgs2/ejercicio5.png)

This figure shows the physical configuration of **Robot 5**, including its links and joints.

---

## Frame Assignment / Scheme
![Robot 5 scheme](imgs2/ejercicio5.1.png)
![](imgs2/ejercicio5.2.png)
![](imgs2/ejercicio5.3.png)
![](imgs2/ejercicio5.4.png)
![](imgs2/ejercicio5.5.png)
![](imgs2/ejercicio5.6.png)


## DH Transformation Matrix
![Robot 5 DH](imgs2/ejercicio5.7.png)
![](imgs2/ejercicio5.8.png)
