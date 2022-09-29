# Lidar Visibility

## Description
This package allows to compute the lidar visibility as described in the paper [On the Importance of Quantifying Visibility for Autonomous Vehicles under Extreme Precipitation](https://arxiv.org/abs/2209.03226). The visibility is defined as the distance at which a lidar beam reaches an object with a certain probability. It is heavily based on the [Lambda mapper package](https://github.com/norlab-ulaval/Lambda_mapper).

Be aware that it is a research code that is still in progress.
Feel free to contact us if you have any question!

## Subscribed topics
- `laser\_scan\_in` (`sensor\_msgs/Laserscan`)
Laser scan to create the map from

## Published topics
- `lambda\_grid\_out` (`msg/lambdaGrid`)  
Get the map data from this topic, which is latched, and updated periodically

- `occupancy\_grid\_out` (`navs\_msgs/OccupancyGrid`) 
Get the map data from this topic, which is latched, and updated periodically

- `visibility\_out` (`std\_msgs/Float32`)
Get the visibility on this topic, updated at each new LaserScan input message.

##  Parameters
In addition to the parameters inherited from [Lambda mapper package](https://github.com/norlab-ulaval/Lambda_mapper) this package uses :

- `min\_range`: minimum distance between sensor and cells used to compute visibility
- `max\_range`: maximum distance between sensor and cells used to compute visibility
- `probability`: probability used to define visibility
