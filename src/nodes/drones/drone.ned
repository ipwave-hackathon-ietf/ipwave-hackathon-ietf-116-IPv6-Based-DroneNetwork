//
// Copyright (C) 2014 OpenSim Ltd.
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

package simu5g.nodes.drones;

//import inet.node.inet.AdhocHost;
import simu5g.nodes.drones.DroneHost;
import inet.routing.aodv.Aodv;

import inet.applications.contract.IApp;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import inet.networklayer.contract.IRoutingTable;
import inet.networklayer.contract.INetworkLayer;
//import inet.networklayer.configurator.ipv4.HostAutoConfigurator;
import inet.networklayer.configurator.ipv6.Ipv6FlatNetworkConfigurator;

import inet.transportlayer.tcp.Tcp;
import inet.transportlayer.udp.Udp;
import inet.common.MessageDispatcher;
import simu5g.nodes.Ue6;
import simu5g.nodes.NR.NRUe6;

//
// It models a ~WirelessHost extended with ~Aodv submodule.
//
// AODV is an on-demand MANET routing protocol. Each node
// works similar to a 'router': they manage their own routing
// tables, send request for routes and consult the IP layer via
// Netfilter hooks for data forwarding.
//
// Scenerios where AODV routing is desired use this module as a
// host module.
//
// The AODV v.2 (known as Dymo) is also implemented: ~Dymo,
// ~DymoRouter.
//
// @see ~Aodv, ~Dymo, ~DymoRouter.
//
module DroneAODVRouter1 extends DroneHost
{
    submodules:
        aodv: Aodv {
            @display("p=825,226");
        }
    connections:
        aodv.socketOut --> at.in++;
        aodv.socketIn <-- at.out++;
}

module UEDrone extends Ue6
{
    parameters:
        @display("i=dronesim/drone");

        //# Network Layer specs
        *.routingTableModule = default(absPath(".ipv6.routingTable"));
   		*.interfaceTableModule = default(absPath(".interfaceTable"));
       
//        ipv4.configurator.typename = "HostAutoConfigurator";
        ipv6.configurator.typename = "Ipv6FlatNetworkConfigurator";
//        ipv4.configurator.interfaces = "cellular lo0";
        ipv6.configurator.interfaces = "cellular lo0";

}

module NRDrone extends NRUe6{
     parameters:

        @display("i=dronesim/drone");
        //# Mobility
//        mobility.typename = default("VeinsInetMobility");

        //# Network Layer specs
        *.routingTableModule = default(absPath(".ipv6.routingTable"));
        
//        ipv4.configurator.typename = "HostAutoConfigurator";
    	ipv6.configurator.typename = "Ipv6FlatNetworkConfigurator";
        
//        ipv4.configurator.interfaces = "cellular lo0";
   		ipv6.configurator.interfaces = "cellular lo0";
        
}