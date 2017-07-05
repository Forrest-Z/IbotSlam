#include "CostMap.h"
#include <omp.h>

CostMap::CostMap(const uint8_t fixedmax, unsigned int size, double resMpPix)
    : GridMap<uint8_t>(size,resMpPix), m_realMax(0), m_max(fixedmax)
{
  memset(m_map,m_max,m_globalSize);
}

CostMap::CostMap(const ProbabilityGridMap &prob, const uint8_t costmax, const uint8_t seuil)
    : GridMap<uint8_t>(prob.size(),prob.resolutionMpPix()), m_max(costmax)
{
    m_realMax = 0;
    //std::////cout << "Le max est " << m_max << std::endl;
    memset(m_map,m_max,m_globalSize);

    std::vector<unsigned int> obstacles;

    uint8_t _seuil = 100 - seuil;
    for(unsigned int i = 0; i < m_globalSize; i++)
      if(prob.at(i) > _seuil)
        obstacles.push_back(i);

    computeALL(&obstacles);
    obstacles.clear();
}

CostMap::~CostMap()
{

}

void CostMap::computeALL(const std::vector<unsigned int>* obstacles) {
    std::vector<unsigned int> currentId = *obstacles, nextId;
    uint8_t currentCost = 1;
    unsigned int        currentPixelId = 0, centerId;
    uint8_t currentPixelValue = 0, currentOrigin;

    //cout << "Je suis ici !!!" << endl;

    //unsigned int avancement = 0;

    // Tant qu'il reste des points à traiter :
    while(currentId.size() != 0 && currentCost < m_max) {

        //avancement += currentId.size();

        //cout << "currentCost " << (int)currentCost << " -> " << currentId.size() << endl;

        if(currentCost > m_realMax)
            m_realMax = currentCost;
        const int s(currentId.size());
        for(int i = 0; i < s; i++)
        {
            centerId = currentId[i];
            currentOrigin = at(centerId);

            // Top :
            if(centerId >= m_size)
            {
                currentPixelId = TopId(centerId);
                currentPixelValue = at(currentPixelId);
                if(currentPixelValue > currentCost) {
                    at(currentPixelId) = currentCost;
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                    nextId.push_back(currentPixelId);
                } else if(currentPixelValue == currentCost) {
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                }

                // Top Left:
                if(centerId%m_size) {
                    currentPixelId = TopLeftId(centerId);
                    currentPixelValue = at(currentPixelId);
                    if(currentPixelValue > currentCost) {
                        at(currentPixelId) = currentCost;
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                        nextId.push_back(currentPixelId);
                    } else if(currentPixelValue == currentCost) {
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                    }
                }

                // Top Right:
                if((centerId+1)%m_size) {
                    currentPixelId = TopRightId(centerId);
                    currentPixelValue = at(currentPixelId);
                    if(currentPixelValue > currentCost) {
                        at(currentPixelId) = currentCost;
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                        nextId.push_back(currentPixelId);
                    } else if(currentPixelValue == currentCost) {
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                    }/* else {
                        // RIEN
                    } */
                }

            }

            // Left:
            if(centerId%m_size)
            {
                currentPixelId = LeftId(centerId);
                currentPixelValue = at(currentPixelId);
                if(currentPixelValue > currentCost) {
                    at(currentPixelId) = currentCost;
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                    nextId.push_back(currentPixelId);
                } else if(currentPixelValue == currentCost) {
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                }/* else {
                        // RIEN
                } */
            }

            // Right:
            if((centerId+1)%m_size)
            {
                currentPixelId = RightId(centerId);
                currentPixelValue = at(currentPixelId);
                if(currentPixelValue > currentCost) {
                    at(currentPixelId) = currentCost;
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                    nextId.push_back(currentPixelId);
                } else if(currentPixelValue == currentCost) {
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                }/* else {
                        // RIEN
                } */
            }

            // Bottom Left:
            if(centerId < m_globalSize-m_size)
            {
                if(centerId%m_size) {
                    currentPixelId = BottomLeftId(centerId);
                    currentPixelValue = at(currentPixelId);
                    if(currentPixelValue > currentCost) {
                        at(currentPixelId) = currentCost;
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                        nextId.push_back(currentPixelId);
                    } else if(currentPixelValue == currentCost) {
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                    }/* else {
                        // RIEN
                } */
                }

                // Bottom :

                currentPixelId = BottomId(centerId);
                currentPixelValue = at(currentPixelId);
                if(currentPixelValue > currentCost) {
                    at(currentPixelId) = currentCost;
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                    nextId.push_back(currentPixelId);
                } else if(currentPixelValue == currentCost) {
                    //currentPixelValue->addOrigins(currentOrigin->origin());
                }/* else {
                        // RIEN
                } */


                // Bottom Right :
                if((centerId+1)%m_size)
                {
                    currentPixelId = BottomRightId(centerId);
                    currentPixelValue = at(currentPixelId);
                    if(currentPixelValue > currentCost) {
                        at(currentPixelId) = currentCost;
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                        nextId.push_back(currentPixelId);
                    } else if(currentPixelValue == currentCost) {
                        //currentPixelValue->addOrigins(currentOrigin->origin());
                    }/* else {
                        // RIEN
                } */
                }
            }
        }

        //cv::imwrite(std::string("/home/vincent/Rover-5-Project/Programmation/dev_Maps/imIn_")+std::to_string(currentCost)+".png",*toCvMat());

        currentId.clear();
        currentId = nextId;
        nextId.clear();
        // On incrémente le coût :
        currentCost += 1;
    }

    for(int i = 0; i < obstacles->size(); i++)
    {
        m_map[obstacles->at(i)] = 0;
    }

}

/*void rOc_CostMap::computeRemoving(const unsigned int id)
{
    std::vector<unsigned int> frontiere;
    std::vector<unsigned int> points, nextPoints;
    points.push_back(id);
    unsigned int currentId, otherId;
    uint8_t currentCost, otherCost;

    unsigned long t1 = rOc_Timestamp::SystemElapsedTime_ms();
    unsigned long t2, t3;

    uint8_t costMin=255;

    //          _________________________________________
    //          ::: RECHERCHE DE LA ZONE A RECALCULER :::

    ////cout << "before while" << endl;
    while(!points.empty())
    {


        //cout << "Debut While :" << endl;
        for(unsigned int i = 0; i < points.size(); i++)
        {
            ////cout << "main loop :" << endl;


            currentId = points[i];
            currentCost = at(currentId);

            if(currentCost == m_max) continue;

            ////cout << "current Cost : " << (int)currentCost << endl;
            at(currentId) = m_max;


            // Top :
            ////cout << "Top" << endl;
            if(currentId >= m_size)
            {
                otherId = TopId(currentId);
                otherCost = at(otherId);
                ////cout << "Top Top " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                if(otherCost <= currentCost) {
                    // Is frontiere :
                    //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                    frontiere.push_back(otherId);
                    if(otherCost < costMin) costMin = otherCost;
                } else {
                    // Is not frontiere :
                    nextPoints.push_back(otherId);
                }

                // Top Left:
                ////cout << "Top Left" << endl;
                if(currentId%m_size) {
                    otherId = TopLeftId(currentId);
                    otherCost = at(otherId);
                    ////cout << "Top Left " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                    if(otherCost <= currentCost) {
                        // Is frontiere :
                        ///cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                        frontiere.push_back(otherId);
                        if(otherCost < costMin) costMin = otherCost;
                    } else {
                        // Is not frontiere :
                        nextPoints.push_back(otherId);
                    }
                }

                // Top Right:
                ////cout << "Top Right" << endl;
                if((currentId+1)%m_size) {
                    otherId = TopRightId(currentId);
                    otherCost = at(otherId);
                    ////cout << "Top Right " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                    if(otherCost <= currentCost) {
                        // Is frontiere :
                        //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                        frontiere.push_back(otherId);
                        if(otherCost < costMin) costMin = otherCost;
                    } else {
                        // Is not frontiere :
                        nextPoints.push_back(otherId);
                    }
                }

            }

            // Left:
            ////cout << "Left" << endl;
            if(currentId%m_size)
            {
                otherId = LeftId(currentId);
                otherCost = at(otherId);
                ////cout << "Left " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                if(otherCost <= currentCost) {
                    // Is frontiere :
                    //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                    frontiere.push_back(otherId);
                    if(otherCost < costMin) costMin = otherCost;
                } else {
                    // Is not frontiere :
                    nextPoints.push_back(otherId);
                }
            }

            // Right:
            ////cout << "Left" << endl;
            if((currentId+1)%m_size)
            {
                otherId = RightId(currentId);
                otherCost = at(otherId);
                ////cout << "Right " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                if(otherCost <= currentCost) {
                    // Is frontiere :
                    //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                    frontiere.push_back(otherId);
                    if(otherCost < costMin) costMin = otherCost;
                } else {
                    // Is not frontiere :
                    nextPoints.push_back(otherId);
                }
            }

            // Bottom Left:
            ////cout << "Bottom Bottom" << endl;
            if(currentId < m_globalSize-m_size)
            {
                ////cout << "Bottom Left" << endl;
                if(currentId%m_size)
                {
                    otherId = BottomLeftId(currentId);
                    otherCost = at(otherId);
                    ////cout << "Bottom Left " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                    if(otherCost <= currentCost) {
                        // Is frontiere :
                        //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                        frontiere.push_back(otherId);
                        if(otherCost < costMin) costMin = otherCost;
                    } else {
                        // Is not frontiere :
                        nextPoints.push_back(otherId);
                    }
                }

                // Bottom :
                ////cout << "Bottom Bottom" << endl;
                otherId = BottomId(currentId);
                otherCost = at(otherId);
                ////cout << "Bottom " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                if(otherCost <= currentCost) {
                    // Is frontiere :
                    //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                    frontiere.push_back(otherId);
                    if(otherCost < costMin) costMin = otherCost;
                } else {
                    // Is not frontiere :
                    nextPoints.push_back(otherId);
                }


                // Bottom Right :
                ////cout << "Bottom Right" << endl;
                if((currentId+1)%m_size)
                {
                    otherId = BottomRightId(currentId);
                    otherCost = at(otherId);
                    ////cout << "Bottom Right " << (int)currentCost << " <= ? " << (int)otherCost << endl;
                    if(otherCost <= currentCost) {
                        // Is frontiere :
                        //cout << "current Cost : " << (int)currentCost << " other " << (int)otherCost << endl;
                        frontiere.push_back(otherId);
                        if(otherCost < costMin) costMin = otherCost;
                    } else {
                        // Is not frontiere :
                        nextPoints.push_back(otherId);
                    }
                }
            }
            ////cout << "fin main loop :" << endl;

        }
        //cout << "fin for : " << nextPoints.size() << " , " << frontiere.size() << endl;

        //cout << "il y a " << nextPoints.size() << " points à venir et déjà " << frontiere.size() << "points de frontiere" << endl;


        //        cout << "avant suppression : " << nextPoints.size() << endl;
        //        // Pour supprimer les doublons :
        //        sort(nextPoints.begin(),nextPoints.end());
        //        unique(nextPoints.begin(), nextPoints.end());
        //        cout << "après suppression : " << nextPoints.size() << endl;

        // On transfère les points futurs dans le tableau des points courrants :
        points.clear();
        points = nextPoints;
        nextPoints.clear();
    }


    t2 = rOc_Timestamp::SystemElapsedTime_ms();
    //cv::imwrite(std::string("/home/vincent/Rover-5-Project/Programmation/dev_Maps/SansLeTrou")+std::to_string(id)+".png",*toCvMat());
    unsigned long t2bis = rOc_Timestamp::SystemElapsedTime_ms();


    //          __________________________________
    //          ::: CALCUL DE LA ZONE SUPPRIME :::

    //cout << "costMin = " << (int)costMin << endl;

    //    unsigned int currentId, otherId;
    //    uint8_t currentCost, otherCost;

    unsigned int treated = 0;

    std::vector<unsigned int>::iterator itFrontiere;

    while(costMin <= m_max)
    {

        unsigned int nbTreated = 0;
        //cout << "debut while" << endl;
        //for(itFrontiere = frontiere.begin(); itFrontiere != frontiere.end(); itFrontiere++)
        for(int i = 0; i < frontiere.size(); i++)
        {
            //cout << "debut for" << endl;
            //currentId = *itFrontiere;
            currentId = frontiere[i];

            //cout << "a : " << currentId << endl;
            currentCost = at(currentId);
            //cout << "b" << endl;
            if(currentCost != costMin) continue;
            //cout << "c" << endl;

            nbTreated++;



            // Top :
            //cout << "Top Top" << endl;
            if(currentId >= m_size)
            {
                //cout << "Top" << endl;
                otherId = TopId(currentId);
                otherCost = at(otherId);
                if(otherCost > currentCost) {
                    at(otherId) = costMin+1;
                    //otherCost->addOrigins(currentOrigin->origin());
                    //frontiere.push_back(otherId);
                }

                // Top Left:
                //cout << "Top Left" << endl;
                if(currentId%m_size) {
                    otherId = TopLeftId(currentId);
                    otherCost = at(otherId);
                    if(otherCost > currentCost) {
                        at(otherId) = costMin+1;
                        //otherCost->addOrigins(currentOrigin->origin());
                        //frontiere.push_back(otherId);
                    }
                }

                // Top Right:
                //cout << "Top Right" << endl;
                if((currentId+1)%m_size) {
                    otherId = TopRightId(currentId);
                    otherCost = at(otherId);
                    if(otherCost > currentCost) {
                        at(otherId) = costMin+1;
                        //otherCost->addOrigins(currentOrigin->origin());
                        //frontiere.push_back(otherId);
                    }
                }

            }

            // Left:
            //cout << "Left" << endl;
            if(currentId%m_size)
            {
                otherId = LeftId(currentId);
                otherCost = at(otherId);
                if(otherCost > currentCost) {
                    at(otherId) = costMin+1;
                    //otherCost->addOrigins(currentOrigin->origin());
                    //frontiere.push_back(otherId);
                }
            }

            // Right:
            //cout << "Right" << endl;
            if((currentId+1)%m_size)
            {
                otherId = RightId(currentId);
                otherCost = at(otherId);
                if(otherCost > currentCost) {
                    at(otherId) = costMin+1;
                    //otherCost->addOrigins(currentOrigin->origin());
                    //frontiere.push_back(otherId);
                }
            }

            // Bottom Left:
            //cout << "Bottom Left" << endl;
            if(currentId < m_globalSize-m_size)
            {
                if(currentId%m_size) {
                    otherId = BottomLeftId(currentId);
                    otherCost = at(otherId);
                    if(otherCost > currentCost) {
                        at(otherId) = costMin+1;
                        //otherCost->addOrigins(currentOrigin->origin());
                        //frontiere.push_back(otherId);
                    }
                }

                // Bottom :
                //cout << "Bottom" << endl;
                otherId = BottomId(currentId);
                otherCost = at(otherId);
                if(otherCost > currentCost) {
                    at(otherId) = costMin+1;
                    //otherCost->addOrigins(currentOrigin->origin());
                    //frontiere.push_back(otherId);
                }


                // Bottom Right :
                //cout << "Bottom Right" << endl;
                if((currentId+1)%m_size)
                {
                    //cout << "a" << endl;
                    otherId = BottomRightId(currentId);
                    //cout << "b" << endl;
                    otherCost = at(otherId);
                    //cout << "c" << endl;
                    if(otherCost > currentCost) {
                        //cout << "d" << endl;
                        at(otherId) = costMin+1;
                        //cout << "e" << endl;
                        //otherCost->addOrigins(currentOrigin->origin());
                        //frontiere.push_back(otherId);
                        //cout << "f" << endl;
                    }
                }

                //cout << "Fin Test" << endl;
            }


            //frontiere.erase(itFrontiere);

        }


        costMin++;
        //cout << "Treated : "<< nbTreated << " New Cost = " << (int)costMin << " : " << frontiere.size() << endl;
        //cv::imwrite(std::string("/home/vincent/Rover-5-Project/Programmation/dev_Maps/computeRemovingTmp")+std::to_string(id)+"("+std::to_string(costMin)+").png",*toCvMat());

    }

    //    for(int i = 0; i < frontiere.size(); i++)
    //        at(frontiere[i]) = m_max;



    t3 = rOc_Timestamp::SystemElapsedTime_ms();

    //cout << "Benchmark : (t2-t1) = "  << t2-t1 << " (t3-t2) : " << t3 - t2bis << endl;
    //cv::imwrite(std::string("/home/vincent/Rover-5-Project/Programmation/dev_Maps/AvecLeTrouBouche")+std::to_string(id)+"(1).png",*toCvMat());

} */
