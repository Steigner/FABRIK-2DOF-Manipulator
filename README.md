## FABRIK 2-DOF Manipulator (SCARA)

```javascript
Software
------------------------------------
| Matlab 2022b
| Octave 7.3.0
```

### Description

This project focuses on the implementation of the **FABRIK (Forward And Backward Reaching Inverse Kinematics)** algorithm. The implemented algorithm is an unmodified version of the [original publication](https://doi.org/10.1016/j.gmod.2011.05.003). It is a simple demonstration of the use of the algorithm on a 2DOF SCARA ABB IRB 910SC manipulator (default).

* IK - FABRIK (Forward And Backward Reaching Inverse Kinematics)
* FK - unmodified D-H (Denavit–Hartenberg) Table
* 2D Plot + Animation
* Plot Workspace Envelope
* Possibility Changing Target in Plot 

Within the application, new targets can be set by clicking into the chart. The program will automatically recalculate the result.

:heavy_exclamation_mark: **The code was also tested in Octave**

Run **./main.m** script in Matlab Desktop / Cloud or from terminal: 

```console
matlab -nosplash -nodesktop -r main
```

### Demo Example

https://user-images.githubusercontent.com/54715463/206893383-c45fb66e-e27f-447c-b0a9-fd46335d917f.mp4

### Future Work

Next steps leads to FABRIK-R (modification FABRIK for robotics arm) and add another programming languages. 

## :information_source: Contacts

:mailbox: m.juricek@outlook.com
