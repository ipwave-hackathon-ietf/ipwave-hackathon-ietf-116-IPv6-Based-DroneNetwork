/*
 * DroneNetMob.h
 *
 *  Created on: June 14, 2021
 *      Author: iotlab_aime
 */

#ifndef __INET_DRONENETMOB_H
#define __INET_DRONENETMOB_H
#include <vector>
#include <algorithm>
#include <iostream>
#include "graphstruct.h"

namespace inet {

/**
 * @brief Random mobility model for a mobile host with a mass.
 * See NED file for more info.
 *
 * @author Emin Ilker Cetinbas, Andras Varga
 */
struct parcel{
    int parcelID;
    double weight;
    int priority;
    double exp_time;
    Coord parceldest;
};
struct dest{
    std::string dstID;
    Coord dstPos;
};
struct Bstion{
    std::string bstID;
    Coord bstPos;
};
void parcelsDefinition (int nparcels);

enum parcelSelection{
    CDPF = 0,        //Closest-Deadline-Parcel-First
    CNPF = 1,       //Closest-Neighbor-Parcel-First
    EPDS = 2,      //Efficient Parcel Delivery Service distance/weight
    RSPF = 3,     //Randomly-Selected-Parcel-First
    HPF  = 4     //Heaviest Parcel First
};

class INET_API DroneNetMob2 : public spacegraph
{
  protected:
    // config (see NED file for explanation)
    cPar *changeIntervalParameter = nullptr;
    cPar *angleDeltaParameter = nullptr;
    cPar *rotationAxisAngleParameter = nullptr;
    cPar *speedParameter = nullptr;
    cPar *numdst = nullptr;
    cPar *ox = nullptr;
    cPar *oy = nullptr;
    cPar *oz = nullptr;

    // state
    Quaternion quaternion;
    simtime_t previousChange;
    Coord sourcePosition;
    Coord destination; //BAM
//    bool flagmovedtodst;
    double droneweightcapacity;
    double droneremainingbattery;
    int selectionMethod;
    std::vector<parcel> MissionParcels;
    std::vector<Coord> MissionPath;

    double deliveryStartTime = 0;
    double deliveryEndTime =   0;
    bool isdestinationset = false;

  protected:
    virtual int numInitStages() const override { return NUM_INIT_STAGES; }

    /** @brief Initializes mobility model parameters. */
    virtual void initialize(int stage) override;

    /** @brief Move the host according to the current simulation time. */
    virtual void move() override;
    void orient() override;

    /** @brief Calculate a new target position to move to. */
    virtual void setTargetPosition() override;

/*    Graph information*/
    vnode mydst;
    vnode mysrc;
    std::vector<vnode> MyPath;
    std::vector<Coord> curpath;
    std::vector<Coord> curtrajectory;
    bool flagisreturn = false;



  public:
    DroneNetMob2();
    virtual double getMaxSpeed() const override;
    void destGen(int ndst);
    void parcelsDefinition (int nparcels);
    std::vector<parcel> droneParcelsSelectionFromSource(int parcelSel);
    Coord missionPathNextDest(Coord curpos);
    Coord missionPathNextDest_n(Coord cp);
    Coord destAssignment();
    void managethedestcongestions();
    std::vector<Coord> droneMissionDestinationSelection();
    void shortestPathTraversal();
    node mBasePos; /*Mission base node*/
    std::vector<vnode> MissionPlanedNodes;/*Nodes to fly for the current Mission*/
    vnode curNode; /*/The node the drone is flying towards;*/
    vnode selectNextFlightDst(vnode curdst); /*return the next Node to fly towards*/
    void destinationHandle(); /*Handle self arrival to the destination based on all other arrivals*/
    Coord MissionPathDefinition();
    std::vector<Coord> MissionTrajectoryDefinition;
    simtime_t nextMoveTime();
    std::vector<dest> HeadingDst; //Assign the way-points that a drone will work through
    Bstion DroneBst; //Each drone move from base station and ends the movement at it.


};

} // namespace inet

#endif // ifndef __INET_DRONENETMOB_H

