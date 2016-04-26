This package implements paper "A Framework for Sparse, Non-linear Least Square Problems on Manifolds"

The original paper has presented necessary class, function and macros declarations. I use these declarations with a few modifications, and then add my own implementations and main function logic. My current implementation works very well on dataset: manhattanOlson500.graph, but a few things need to be improved. 

I wish this piece of code can serve as a toy code for people who want to understand g2o and general non linear least square problems better. 


This program depends on eigen3 and gnuplot(to plot the data)


--2016-04-15-

Now I can draw the data, but sparse block matrix becomes a headache


--2016-04-26-

The current version works very well on dataset: manhattanOlson500.graph.

However, I still have problem with Eigen::SparseQR sparse matrix solver. It runs very slow now, but if I switch to SimplicialLLT, the result looks very wrong. My next step should be a careful treatment to the sparse matrix solver.

Now I can get good result for 500 variables in less than 1 second, but for problem with 3500 variables, each optimizationStep takes nearly 5 minutes. 

Following are the photos I currently get for 3500 variables.

Initial guess:
![Inital](https://raw.githubusercontent.com/paulyang1990/toy_code/master/NonliearOpt/img/result1.png)
Final optimized result:
![Final](https://raw.githubusercontent.com/paulyang1990/toy_code/master/NonliearOpt/img/result2.png)
