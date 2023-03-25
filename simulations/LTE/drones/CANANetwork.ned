//
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
package simu5g.simulations.LTE.drones;
import inet.node.inet.AdhocHost;
//import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.physicallayer.wireless.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import simu5g.nodes.drones.DroneAODVRouter1;
//import simu5g.nodes.drones.DroneHost1;
//import inet.visualizer.integrated.IntegratedVisualizer;
import inet.visualizer.common.IntegratedVisualizer;
import simu5g.nodes.drones.Dest;
import simu5g.nodes.drones.MissionBase;

network CANANetwork
{
    parameters:
        int numdrones = default (30);
        int numdst = default (5);
        int nbst =  default (1);
        @display("bgb=600,400");
    submodules: 
        drone[numdrones]: DroneAODVRouter1 {
            parameters:
                @display("p=20,20;r=30,,#707070");
        }
//   		drone[numdrones]: DroneHost1 {
//            parameters:
//                @display("p=20,20;r=30,,#707070");
//        }
		radioMedium: Ieee80211ScalarRadioMedium {
            @display("p=91,35");
        }
        routingTableRecorder: RoutingTableRecorder {
            parameters:
                @display("p=91,100");
        }
		gcs: AdhocHost {
            @display("i=device/antennatower_vl;r=150,,#707070;p=91,200");
        }
        configurator: Ipv4NetworkConfigurator {
            config = xml("<config><interface hosts='*' address='192.168.0.x' netmask='255.255.255.0'/></config>");
            @display("p=91,300");
        }
        dst[numdst]: Dest {
            @display("i=misc/town;r=,,#707070;p=91,380");
        }
        bstion[nbst]: MissionBase{
        	 @display("i=dronesim/bstation");
        }
   	    visualizer:IntegratedVisualizer{
			@display("p=91,20");
    	} 
}
