# COMMES_MLD
A control-oriented modelling tool for multi-energy systems using the mixed-logical dynamical systems framework. It was developed by Michael Taylor at the University of Manchester. It was inspired by the work of Long [1] and Moser et al. [2].

COMMES_MLD is implemented in MATLAB and used for rapid development of large, hybrid, multi-energy system models that can be passed to commercial optimisation solvers. It is therefore suitable for solving dispatch, scheduling and/or model predictive control (MPC) problems.

COMMES_MLD is highly dependent on the free MATLAB software package YALMIP created by Johan Löfberg [3] and the latest release which has been tested for compatability is R20200930 (see https://yalmip.github.io/). YALMIP is used to create, organise and analyse the model variables, constraints and objectives before parsing these inputs for use with the chosen optmisation solver (Gurobi is recommended). Significant overhead is incurred by YALMIP as a result and for simulations requiring repeated optimisations (e.g. MPC) it is recommended to use the YALMIP "optimizer" function. This creates an "optimizer" object with a pre-compiled low-level numerical format that is fixed during simulation. All models produced using COMMES_MLD are intended to work with an optimizer object and, when appropriate, require varying inputs to implement dynamic model constraints (see for example COMMES_MLD/Unit Tests/chp_min_up_down_test.m). See the Component base class for methods that are common to all child classes, e.g. the ability to associate a cost to a decision variable in the objective function, using the '.add_val_cost' method.

All folders must be saved to the MATLAB path before running any examples or developing new models. A series of unit tests may be run using the script "run_unit_tests.m" to ensure that COMMES_MLD is working correctly. The most likely cause of errors will be the lack of a compatible optimisation solver or YALMIP installation.

Contributions, suggestions and bug reports are welcome, please raise a new pull request or issue.

COMMES_MLD is provided under the GPL-3.0 licence.

[1] S. Long, “Generalised Modelling Framework for Muli-Energy Systems With Model Predictive Control Applications,” Ph.D. dissertation, University of Manchester, 2019.

[2] A. Moser, D. Muschick, M. G¨olles, P. Nageler, H. Schranzhofer, T. Mach, C. Ribas Tugores, I. Leusbrock, S. Stark, F. Lackner, and A. Hofer, “A MILP-based modular energy management system for urban multi-energy systems: Performance and sensitivity analysis,” Applied Energy, 2020.

[3] J. L¨ofberg, “Yalmip : A toolbox for modeling and optimization in matlab,” in In Proceedings of the CACSD Conference, Taipei, Taiwan, 2004.
