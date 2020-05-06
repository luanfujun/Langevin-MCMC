# Langevin-MCMC
This is the code repository for paper "[Langevin Monte Carlo Rendering with Gradient-based Adaptation](https://research.cs.cornell.edu/langevin-mcmc/data/paper.pdf)" by Fujun Luan, Shuang Zhao, Kavita Bala, and Ioannis Gkioulekas. Check out the [project page](https://research.cs.cornell.edu/langevin-mcmc/) for more information. 
 
## LMC (44.69 seconds):
![LMC](scenes/torus/lmc_timeuse_44.689152s.png)
## H2MC (45.38 seconds):
![H2MC](scenes/torus/h2mc_timeuse_45.381592s.png)

We implement our proposed LMC algorithms within the open-source reference implementation of H2MC by Tzu-Mao Li. Please refer to the dpt renderer page for basic installation and usage: https://github.com/BachiLi/dpt.  

If you use our code, please cite the paper:
```bibtex
@article{luan2020langevin,
  title = {Langevin Monte Carlo Rendering with Gradient-based Adaptation},
  author = {Luan, Fujun and Zhao, Shuang and Bala, Kavita and Gkioulekas, Ioannis},
  journal = {ACM Trans. Graph.},
  volume = {39},
  number = {4},
  year = {2020},
  publisher = {ACM},
}
```

The code has been tested on a 32-core machine with Ubuntu 18.04 and gcc 9.2.1. I have not tested it on OSX or Windows system yet.

Please feel free to contact me (fujun @ cs.cornell.edu) if there are any issues/comments/questions.
