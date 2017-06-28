#ifndef DEFINE_H
#define DEFINE_H

#define   TOPIC_LiDAR       "scan"
#define   TOPIC_

//    _____________________________
//    ::: Définition des frames :::
#define   FRAME_BASE        "base_link"
#define   FRAME_MAP         "map"

//    _____________________________________
//    ::: Définition des noms de Topics :::
#define   TOPIC_SCAN        "scan"
#define   TOPIC_POSE        "poseSlamOmatic"
#define   TOPIC_MAP         "mapSlamOmatic"
#define   TOPIC_PATH        "pathSlamOmatic"

//    _________________________________
//    ::: Configuration de la carte :::

// Taille en pixel de la carte :
#define   MAP_SIZE          3500

// Résolution de la carte en metre par pixel
#define   MAP_RESOLUTION    0.02

// Cout max possible en pixel autour des obstacles dans la carte de cout
#define   MAP_COSTMAX       45

// Seuil pour extraire les obstacles et placer dans la carte de cout
#define   MAP_SEUIL         45

// Interval de recherche sur X, Y, Théta formant le simplexe du Nelder & Mead
#define   INTER_SEARCH      0.05f     // en mètre

//    ::: Configuration de la carte de probabilité :::
// Valeur ajoutée/soustraite lorsque l'on modifie la carte de probabilité :
#define   PROBA_INCREMENT   5
#define   PROBA_DECREMENT   1

//    ::: FILTRAGE :::

// Seuil au delà du quel on ne tient pas compte des points LiDAR dans la localisation
#define   NEWS_FILTER       1020.2f

// Distance maximale en metre entre deux points consécutifs dans un scan LiDAR, tous les autres point sont supprimés
#define   SEUIL_DISTANCE    0.00f

#define   SEUIL_DIST_CARTO  4000.0f





























#endif // DEFINE_H

///       ----------------------------------
///       <-- POUR LES CARTES DU RPLIDAR -->
///       ----------------------------------
/*
#define   MAP_SIZE          3501
#define   MAP_COSTMAX       45
#define   MAP_SEUIL         45
#define   MAP_RESOLUTION    0.04

#define   INTER_SEARCH      0.1     // en mètre
*/
///       ------------------------------

///       ------------------------------
///       <-- POUR LES CARTES DU MIT -->
///       ------------------------------

//#define   MAP_SIZE          6001
//#define   MAP_COSTMAX       45
//#define   MAP_SEUIL         45
//#define   MAP_RESOLUTION    0.02

//#define   INTER_SEARCH      0.05     // en mètre

///       ------------------------------


//#define   LOC_SEARCH_X_PX     0.2    // 7.5pix
//#define   LOC_SEARCH_Y_PX     0.2    // 7.5pix

//#ifdef  USE_IMU
//#define   LOC_SEARCH_T_DEG    3
//#else
//#define   LOC_SEARCH_T_DEG    1
//#endif
