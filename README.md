## FABRIK 2-DOF Manipulator (SCARA)

```javascript
Software
------------------------------------
| Matlab 2022b
```

### Description

This project focuses on the implementation of the **FABRIK (Forward And Backward Reaching Inverse Kinematics)** algorithm. The implemented algorithm is an unmodified version of the [original publication](https://doi.org/10.1016/j.gmod.2011.05.003). It is a simple demonstration of the use of the algorithm on a 2DOF SCARA ABB IRB 910SC manipulator (default).

* IK - FABRIK (Forward And Backward Reaching Inverse Kinematics)
* FK - unmodified D-H (Denavit–Hartenberg) Table
* 2D Plot + Animation
* Plot Workspace Envelope
* Possibility Changing Target in Plot 

Within the application, new targets can be set by clicking into the chart. The program will automatically recalculate the result.

Run **./main.m** script in Matlab Desktop / Cloud or from terminal: 

```console
matlab -nosplash -nodesktop -r main
```

### Demo Example

[Demo Example.webm](https://user-images.githubusercontent.com/54715463/206244516-66396223-90b3-49d5-9f85-70da7663866e.webm)

### Future Work

Next steps leads to FABRIK-R (modification FABRIK for robotics arm) and add another programming languages. 

## :information_source: Contacts

:mailbox: m.juricek@outlook.com
