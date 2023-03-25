// 
//                  Simu5G
//
// Authors: Giovanni Nardini, Giovanni Stea, Antonio Virdis (University of Pisa)
// 
// This file is part of a software released under the license included in file
// "license.pdf". Please read LICENSE and README files before using it.
// The above files and the present reference are part of the software itself, 
// and cannot be removed from it.
//
package simu5g.simulations.NR.carsdronesipv6;

//import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.configurator.ipv6.Ipv6FlatNetworkConfigurator;

import inet.networklayer.ipv6.Ipv6RoutingTable;
import inet.node.inet.AdhocHost;
//import inet.node.inet.Router;
import inet.node.ipv6.Router6;

//import inet.node.inet.StandardHost;
import inet.node.ethernet.Eth10G;
//import inet.node.xmipv6.WirelessHost6;
import inet.node.ipv6.StandardHost6;

import simu5g.world.radio.LteChannelControl;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf6;
import simu5g.common.binder.Binder;
import simu5g.nodes.NR.gNodeB6;
import simu5g.nodes.cars.NRCar2;
import simu5g.nodes.drones.NRDrone;

import org.car2x.veins.subprojects.veins_inet.VeinsInetManager;

network HighwayNet
{
    parameters:
        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
		int numdrones = default (1);
        @display("bgb=732,483");

    submodules:
//
//        routingRecorder: Ipv6RoutingTable {
//            @display("p=50,75;is=s");
//  		 }
       configurator: Ipv6FlatNetworkConfigurator {
            @display("p=50,125");
        }

        //# Veins manager module
        veinsManager: VeinsInetManager {
            @display("p=50,227;is=s");
        }

        //# LTE modules
        channelControl: LteChannelControl {
            @display("p=50,25;is=s");
        }
        binder: Binder {
            @display("p=50,175;is=s");
        }
        carrierAggregation: CarrierAggregation {
            @display("p=50.993748,258.7;is=s");
        }
//        server: StandardHost {
//            @display("p=660,136;is=n;i=device/server");
//        }
        server: StandardHost6 {
            @display("p=660,136;is=n;i=device/server");
        }
        router: Router6 {
            @display("p=561,135;i=device/smallrouter");
        }
        upf: Upf6 {
            @display("p=462,136");
        }
        gNodeB1: gNodeB6 {
            @display("p=156,136;is=vl");
        }
        gNodeB2: gNodeB6 {
            @display("p=391,313;is=vl");
        }
        //# submodule vector to store all cars - will be filled dynamically during simulations
        car[0]: NRCar2;
        drone[numdrones]: NRDrone {
            parameters:
               @display("p=20,20;r=30,,#707070");
        }
    connections allowunconnected:
        server.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> gNodeB1.ppp;
        upf.pppg++ <--> Eth10G <--> gNodeB2.ppp;

        //# X2 connections
        gNodeB1.x2++ <--> Eth10G <--> gNodeB2.x2++;
}

