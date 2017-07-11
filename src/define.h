#ifndef DEFINE_H
#define DEFINE_H

//    ________________
//    ::: Frame Id :::
#define   FRAME_BASE        "base_link"
#define   FRAME_MAP         "map"

//    ____________________
//    ::: Topics names :::
#define   TOPIC_SCAN        "scan"
#define   TOPIC_POSE        "poseSlamOmatic"
#define   TOPIC_MAP         "mapSlamOmatic"
#define   TOPIC_PATH        "pathSlamOmatic"

//    _____________________
//    ::: Service names :::
#define   SERVICE_PROBMAP   "slam_prob_map"
#define   SERVICE_COSTMAP   "slam_cost_map"

//    __________________________
//    ::: Mapping parameters :::

// Map size in pixel
#define   MAP_SIZE          3500

// Map resolution in meters by pixel
#define   MAP_RESOLUTION    0.02

// Maximal cost encoded in the costmap
#define   MAP_COSTMAX       45

// Threshold apply to the probabilistic map
#define   MAP_SEUIL         45

// Research interval on X, Y and Theta for the simplex in Nelder & Mead algorithm
#define   INTER_SEARCH      0.05f     // en mètre

// Modification values on probabilistic map :
#define   PROBA_INCREMENT   5
#define   PROBA_DECREMENT   1

//    ::: FILTRAGE :::

// Threshold for new points in meter :
#define   NEWS_FILTER       1020.2f

#endif // DEFINE_H
