// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

package simu5g.simulations.NR.drones;


import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.inet.AdhocHost;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import inet.node.ethernet.Eth10G;

import simu5g.world.radio.LteChannelControl;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.common.binder.Binder;
import simu5g.nodes.NR.gNodeB;
import simu5g.nodes.cars.NRCar;
import simu5g.nodes.drones.NRDrone;
import simu5g.nodes.drones.Dest;
import simu5g.nodes.drones.MissionBase;



network NRCANANetwork
{
    parameters:
//        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
//        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
//        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)

        double playgroundSizeX @unit(m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m); // z size of the area the nodes are in (in meters)
        @display("bgb=732,483");
       	int numdrones = default (30);
        int numdst = default (5);
        int nbst =  default (1);
//        @display("bgb=600,400");
    submodules:

        drone[numdrones]: NRDrone {
            parameters:
                @display("p=20,20;r=30,,#707070");
        }
        routingRecorder: RoutingTableRecorder {
            @display("p=50,75;is=s");
        }
        configurator: Ipv4NetworkConfigurator {
            @display("p=50,125");
            config = xmldoc("demo.xml");
        }
        dst[numdst]: Dest {
            @display("i=misc/town;r=,,#707070;p=91,380");
        }
        bstion[nbst]: MissionBase{
        	 @display("i=dronesim/bstation");
        }
//		gcs: AdhocHost {
//            @display("i=device/antennatower_vl;r=150,,#707070;p=91,200");
//        }
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
        server: StandardHost {
            @display("p=660,136;is=n;i=device/server");
        }
        router: Router {
            @display("p=561,135;i=device/smallrouter");
        }
        upf: Upf {
            @display("p=462,136");
        }
        gNodeB1: gNodeB {
            @display("p=156,136;is=vl");
        }
        gNodeB2: gNodeB {
            @display("p=391,313;is=vl");
        }
    connections allowunconnected:
        server.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> gNodeB1.ppp;
        upf.pppg++ <--> Eth10G <--> gNodeB2.ppp;

        //# X2 connections
        gNodeB1.x2++ <--> Eth10G <--> gNodeB2.x2++;
}
