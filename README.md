## Snake

Snake is a set of pybullet/pyrosim-based functions that evolves a snake-like robot comprised of random rectangular pieces. Links are sensorized at random, while joints are motorized at random. A sensing-enabled link is distinguished from a non-sensing link by its color; the former is green, while the latter is blue. 

In brief, the snake algorithm works by defining a head for the snake and adding anywhere from 1 to 8 pieces. The pieces' dimensions (length, width, and height) range from 0.1 to 3.0 units. 


## Usage

Please make sure all of the attending files from this repository are present in your working directory (e.g., by cloning this repo to your local system).

Run the following in the command line:

```bash
python3 snakeSim.py
```


## Output

The the simulation is run 5 times to produce 5 unique, random snakes. This value can be changed in ```snakeSim.py``` (in the for loop) if more iterations of simulations are desired. 


## Citations
Bongard, J. [u/DrJosh]. “Education in Evolutionary Robotics” Reddit, 6 Feb. 2023, https://www.reddit.com/r/ludobots/.

Kriegman, S. Artificial Life, Northwestern University, Evanston, Illinois, Winter 2023. 
