<p align="center">
<img alt="Architecture" src="core_navigation_demos/doc/corenav4.gif" width="300">
</p>

## Overview

**Author: Cagri Kilic<br />
Affiliation: [WVU NAVLAB](https://navigationlab.wvu.edu/)<br />
Maintainer: Cagri Kilic, cakilic@mix.wvu.edu**

Improved Planetary Rover Inertial Navigation and Wheel Odometry Performance through Periodic Use of Zero-Type Constraint

<p align="center">
<img alt="Architecture" src="core_navigation_demos/doc/architecturev2.jpg" width="700">
</p>

This repository provides a localization algorithm that compensates for the high likelihood of odometry errors by providing a reliable localization solution that leverages nonholonomic vehicle constraints as well as state-aware pseudo-measurements (e.g., zero velocity and zero angular rate) updates during periodic stops.


**Keywords:** localization, dead-reckoning, zero-velocity, non-holonomic


## Citation

If you find this library useful, please cite the following publication:

* Cagri Kilic, Jason N. Gross, Nicholas Ohi, Ryan Watson, Jared Strader, Thomas Swiger, Scott Harper, and Yu Gu: **Improved Planetary Rover Inertial Navigation and Wheel Odometry Performance through Periodic Use of Zero-Type Constraints**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2019. ([arXiv:1906.08849](https://arxiv.org/pdf/1906.08849.pdf))

        @inproceedings{Kilic2019,
            author = {Kilic, Cagri and Gross,Jason N. and Ohi,Nicholas and Watson, Ryan and Strader, Jared and Swiger, Thomas and Harper, Scott and Gu, Yu},
            booktitle = {2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
            title = {{Improved Planetary Rover Inertial Navigation and Wheel Odometry Performance through Periodic Use of Zero-Type Constraints}},
            year = {2019},
            pages={552-559},
            doi={10.1109/IROS40897.2019.8967634},
            ISSN={2153-0858}, 
            publisher = {IEEE},
            
        }


## IROS Figures

To generate the same figures in the paper, use [v.1.0](https://github.com/wvu-navLab/CLN/tree/v1.0) and simply run MainRun.m
