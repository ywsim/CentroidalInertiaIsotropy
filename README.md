# Centroidal Inertia Isotropy

### Introduction and Motivation 

The controllers of dynamic legged robots such as Cassie, MIT Mini Cheetah, or Atlas employ planners based on reduced models such as variations of linear inverted pendulum (LIP) model, spring loaded inverted pendulum (SLIP), single rigid body model (SRBM), or centroidal dynamics.
However, the actual dynamics of legged robots differs from these simple models, because the mass of the limbs contributes to inertia of the whole system. 
To reduce this difference, aforementioned robots are built with lightweight limbs to resemble simpler models. 
For example, actuators, which contributes to large part of the total movingmass, are placed near the center of mass or torso (Proximal Actuation).
Moreover, smaller gap between the dynamics of model and real robot promotes more efficienct control synthesis. 
For instance, in case of mini cheetah, its dynamics was approximated as a single rigid body. This simplification enabled efficienct MPC controller sythesis. 

We introduce a design metric called Centroidal Inertia Isotropy (CII), to quantify the state of proximodistal mass distribution as a caliber of proximal actuation. 
CII can compare a design scenario accorss different robots such as biped, quadruped or animaloid. This is because CII can be calculated for any robot represented as  a rigid body structure. 

### What you can do with this repo
`tutorial.m`
1. Create rigid body struture of 3 different tpyes of robots using URDF files 
    - Quadruped (Mini Cheetah)
    - Biped, Proximal Actuation (Tello, Cassie, Atlas (DRC))
    - Biped, Collocated Actuation (Tello, HuboPlus)
2. Visualize robots using mesh files (Matlab Robotics Toolbox required)
3. Calculate centroidal inertia matrix
4. Calculate centroidal inertia isotropy
5. Compare the proximodistal mass distribution of various robots
<img src=https://user-images.githubusercontent.com/25633568/169211860-7c26c113-9f7b-4cf6-8f24-d1f8f3a48fcc.jpg width="400">

`RAL2022_paper_figures.m`
1. Generate original figures on the RAL2022 paper (Link: TBU)
<p float="left">
<img src=https://user-images.githubusercontent.com/25633568/169211854-92877a2f-0279-4ae9-8211-c4da2ffde718.png height="300">
<img src=https://user-images.githubusercontent.com/25633568/169211859-672b5208-056b-42fa-aadf-d57a4a40314d.png height="300">
</p>

### Code overview
- `RBDyn3 (class)`
    - RBDyn3 creates robot object by 1. importing URDF and 2. associating 1. with Featherstone's rigid body dynamics algorithm
- `obj.getFullConfig([q2, q3, q4])`
    - It is assumed that users can play with three joint angles; Hip Ab/Adduction (q2), Hip Flex/Extension (q3), and Knee Flex/Extension (q4). 
    - However, actual robot models have more joints. So, to conform to inidividual robot's configuration, the above three joint angles need to be expanded to full configuration vector using 'getFullConfig'. 
- `obj.calcCII(q0, q)`
    - CII is calculated per configuration, q. Plus, CII is calculated with respect to a nominal configuation q0. 
    - For instance, calcCII(q0, q0) is always zero
    - CII can be evaluated for all joint configurations within joint limits. (There is no self collision detection)
    - Among those CII values, there are max and min values. We are particularly interested in rCII:=maxCII - minCII.


### Caveats
1. The URDF files are modified; a world body is inserted before torso (base body) to enable floating-based model.  
2. When importing URDF files, it associates paramters and the tree structure with Featherstone's rigid body dynamics algorithm. 
3. To calculation of centroidal inertia was added on top of the original Featherstone's library ('getCentrInertia' method)
