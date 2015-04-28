"""
@file    __init__.py
@author  Daniel Krajzewicz
@author  Laura Bieker
@author  Karol Stosiek
@author  Michael Behrisch
@author  Jakob Erdmann
@date    2008-03-27
@version $Id: __init__.py 17235 2014-11-03 10:53:02Z behrisch $

This file contains a content handler for parsing sumo network xml files.
It uses other classes from this module to represent the road network.

SUMO, Simulation of Urban MObility; see http://sumo.dlr.de/
Copyright (C) 2008-2014 DLR (http://www.dlr.de/) and contributors

This file is part of SUMO.
SUMO is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 3 of the License, or
(at your option) any later version.
"""

from __future__ import print_function
import os, sys
import math
import random
from xml.sax import saxutils, parse, handler
from copy import copy
from itertools import *

import sumolib
from . import lane, edge, node, connection, roundabout
from .lane import Lane
from .edge import Edge
from .node import Node
from .connection import Connection
from .roundabout import Roundabout

# for scheduler
import numpy as np
#sys.path.append('/Users/hsuchieh/Documents/SUMO/python-constraint-1.2')
from .constraint_rushhour import *
import xml.etree.cElementTree as ET
import operator


class TLS:
    """Traffic Light Signal for a sumo network"""
    def __init__(self, id):
        self._id = id
        self._connections = []
        self._maxConnectionNo = -1
        self._programs = {}

    def addConnection(self, inLane, outLane, linkNo):
        self._connections.append( [inLane, outLane, linkNo] )
        if linkNo>self._maxConnectionNo:
            self._maxConnectionNo = linkNo

    def getConnections(self):
        return self._connections

    def getID(self):
        return self._id

    def getLinks(self):
        links = {}
        for connection in self._connections:
            if connection[2] not in links:
                links[connection[2]] = []
            links[connection[2]].append(connection)
        return links

    def getEdges(self):
        edges = set()
        for c in self._connections:
            edges.add(c[0].getEdge())
        return edges

    def addProgram(self, program):
        self._programs[program._id] = program

    def toXML(self):
        ret = ""
        for p in self._programs:
            ret = ret + self._programs[p].toXML(self._id)
        return ret 


class TLSProgram:
    def __init__(self, id, offset, type):
        self._id = id
        self._type = type
        self._offset = offset
        self._phases = []

    def addPhase(self, state, duration):
        self._phases.append( (state, duration) )

    def toXML(self, tlsID):
        ret = '  <tlLogic id="%s" type="%s" programID="%s" offset="%s">\n' % (tlsID, self._type, self._id, self._offset)
        for p in self._phases:
            ret = ret + '    <phase duration="%s" state="%s"/>\n' % (p[1], p[0])
        ret = ret + '  </tlLogic>\n'
        return ret


class Net:
    """The whole sumo network."""
    def __init__(self):
        self._location = {}
        self._id2node = {}
        self._id2edge = {}
        self._crossings_and_walkingAreas = set()
        self._id2tls = {}
        self._nodes = []
        self._edges = []
        self._tlss = []
        self._ranges = [ [10000, -10000], [10000, -10000] ]
        self._roundabouts = []
        self._rtree = None
        self._allLanes = []

    def setLocation(self, netOffset, convBoundary, origBoundary, projParameter):
        self._location["netOffset"] = netOffset
        self._location["convBoundary"] = convBoundary
        self._location["origBoundary"] = origBoundary
        self._location["projParameter"] = projParameter
        
    def addNode(self, id, type=None, coord=None, incLanes=None):
        if id not in self._id2node:
            n = node.Node(id, type, coord, incLanes)
            self._nodes.append(n)
            self._id2node[id] = n
        self.setAdditionalNodeInfo(self._id2node[id], type, coord, incLanes)
        return self._id2node[id]
    
    def setAdditionalNodeInfo(self, node, type, coord, incLanes):
        if coord!=None and node._coord==None:
            node._coord = coord
            self._ranges[0][0] = min(self._ranges[0][0], coord[0])
            self._ranges[0][1] = max(self._ranges[0][1], coord[0])
            self._ranges[1][0] = min(self._ranges[1][0], coord[1])
            self._ranges[1][1] = max(self._ranges[1][1], coord[1])
        if incLanes!=None and node._incLanes==None:
            node._incLanes = incLanes
        if type!=None and node._type==None:
            node._type = type

    def addEdge(self, id, fromID, toID, prio, function, name):
        if id not in self._id2edge:
            fromN = self.addNode(fromID)
            toN = self.addNode(toID)
            e = edge.Edge(id, fromN, toN, prio, function, name)
            self._edges.append(e)
            self._id2edge[id] = e
        return self._id2edge[id]

    def addLane(self, edge, speed, length, allow=None, disallow=None):
        return lane.Lane(edge, speed, length, allow, disallow)

    def addRoundabout(self, nodes):
        r = roundabout.Roundabout(nodes)
        self._roundabouts.append(r)
        return r

    def addConnection(self, fromEdge, toEdge, fromlane, tolane, direction, tls, tllink):
        conn = connection.Connection(fromEdge, toEdge, fromlane, tolane, direction, tls, tllink)
        fromEdge.addOutgoing(conn)
        fromlane.addOutgoing(conn)
        toEdge._addIncoming(conn)

    def getEdges(self):
        return self._edges

    def getRoundabouts(self):
        return self._roundabouts

    def hasEdge(self, id):
        return id in self._id2edge

    def getEdge(self, id):
        return self._id2edge[id]

    def _initRTree(self, shapeList, includeJunctions=True):
        import rtree
        self._rtree = rtree.index.Index()
        self._rtree.interleaved = True
        for ri, shape in enumerate(shapeList):
            self._rtree.add(ri, shape.getBoundingBox(includeJunctions))

    def getNeighboringEdges(self, x, y, r=0.1, includeJunctions=True):
        edges = []
        try:
            if self._rtree == None:
                self._initRTree(self._edges, includeJunctions)
            for i in self._rtree.intersection((x - r, y - r, x + r, y + r)):
                e = self._edges[i]
                d = sumolib.geomhelper.distancePointToPolygon((x,y), e.getShape(includeJunctions))
                if d < r:
                    edges.append((e, d))
        except ImportError:
            for edge in self._edges:
                d = sumolib.geomhelper.distancePointToPolygon((x,y), edge.getShape(includeJunctions))
                if d < r:
                    edges.append((edge,d))
        return edges

    def getNeighboringLanes(self, x, y, r=0.1, includeJunctions=True):
        lanes = []
        try:
            if self._rtree == None:
                if not self._allLanes:
                    for edge in self._edges:
                        self._allLanes += edge.getLanes()
                self._initRTree(self._allLanes, includeJunctions)
            for i in self._rtree.intersection((x - r, y - r, x + r, y + r)):
                l = self._allLanes[i]
                d = sumolib.geomhelper.distancePointToPolygon((x,y), l.getShape(includeJunctions))
                if d < r:
                    lanes.append((l, d))
        except ImportError:
            for edge in self._edges:
                for l in edge.getLanes():
                    d = sumolib.geomhelper.distancePointToPolygon((x,y), l.getShape(includeJunctions))
                    if d < r:
                        lanes.append((l, d))
        return lanes

    def hasNode(self, id):
        return id in self._id2node

    def getNode(self, id):
        return self._id2node[id]

    def getNodes(self):
        return self._nodes

    def getTLSSecure(self, tlid):
        if tlid in self._id2tls:
            tls = self._id2tls[tlid]
        else:
            tls = TLS(tlid)
            self._id2tls[tlid] = tls
            self._tlss.append(tls)
        return tls

    def addTLS(self, tlid, inLane, outLane, linkNo):
        tls = self.getTLSSecure(tlid)
        tls.addConnection(inLane, outLane, linkNo)
        return tls

    def addTLSProgram(self, tlid, programID, offset, type):
        tls = self.getTLSSecure(tlid)
        program = TLSProgram(programID, offset, type)
        tls.addProgram(program)
        return program


    def setFoes(self, junctionID, index, foes, prohibits):
        self._id2node[junctionID].setFoes(index, foes, prohibits)

    def forbids(self, possProhibitor, possProhibited):
        return possProhibitor[0].getEdge()._to.forbids(possProhibitor, possProhibited)

    def getDownstreamEdges(self, edge, distance, stopOnTLS):
        ret = []
        seen = set()
        toProc = []
        toProc.append( [edge, 0, [] ] )
        while not len(toProc)==0:
            ie = toProc.pop()
            if ie[0] in seen:
                continue
            seen.add(ie[0])
            if ie[1] + ie[0].getLength() >= distance:
                ret.append( [ie[0], ie[0].getLength()+ie[1]-distance, ie[2], False] )
                continue
            if len(ie[0]._incoming)==0:
                ret.append( [ie[0], ie[0].getLength()+ie[1], ie[2], True] )
                continue
            mn = []
            hadTLS = False
            for ci in ie[0]._incoming:
                if ci not in seen:
                    prev = copy(ie[2])
                    if stopOnTLS and ci._tls and ci!=edge and not hadTLS:
                        ret.append( [ie[0], ie[1], prev, True ] )
                        hadTLS = True
                    else:
                        prev.append(ie[0])
                        mn.append( [ci, ie[0].getLength()+ie[1], prev ] )
            if not hadTLS:
                toProc.extend(mn)
        return ret

    # the diagonal of the bounding box of all nodes
    def getBBoxDiameter(self):
        return math.sqrt(
                (self._ranges[0][0] - self._ranges[0][1]) ** 2 +
                (self._ranges[1][0] - self._ranges[1][1]) ** 2)

    def getGeoProj(self):
        import pyproj
        p1 = self._location["projParameter"].split()
        params= {}
        for p in p1:
          ps = p.split("=")    
          if len(ps)==2:
            params[ps[0]] = ps[1]
          else:
            params[ps[0]] = True
        return pyproj.Proj(projparams=params)

    def getLocationOffset(self):
        """ offset to be added after converting from geo-coordinates to UTM"""
        return list(map(float,self._location["netOffset"].split(",")))

    def convertLonLat2XY(self, lon, lat, rawUTM=False):
        x, y = self.getGeoProj()(lon, lat)
        if rawUTM:
            return x, y
        else:
            x_off, y_off = self.getLocationOffset()
            return x + x_off, y + y_off

    def convertXY2LonLat(self, x, y, rawUTM=False):
        if not rawUTM:
            x_off, y_off = self.getLocationOffset()
            x -= x_off
            y -= y_off
        return self.getGeoProj()(x, y, inverse=True)

    def move(self, dx, dy):
        for n in self._nodes:
            n._coord = (n._coord[0] + dx, n._coord[1] + dy)
        for e in self._edges:
            for l in e._lanes:
                l._shape = [(p[0] + dx, p[1] + dy) for p in l._shape]
            e.rebuildShape()


class NetReader(handler.ContentHandler):
    """Reads a network, storing the edge geometries, lane numbers and max. speeds"""

    def __init__(self, **others):
        self._net = others.get('net', Net())
        self._currentEdge = None
        self._currentNode = None
        self._currentLane = None
        self._currentShape = ""
        self._withPhases = others.get('withPrograms', False)
        self._withConnections = others.get('withConnections', True)
        self._withFoes = others.get('withFoes', True)

    def startElement(self, name, attrs):
        if name == 'location':
            self._net.setLocation(attrs["netOffset"], attrs["convBoundary"], attrs["origBoundary"], attrs["projParameter"])
        if name == 'edge':
            function = attrs.get('function', '') 
            if function == '':
                prio = -1
                if attrs.has_key('priority'):
                    prio = int(attrs['priority'])
                name = ""
                if attrs.has_key('name'):
                    name = attrs['name']
                self._currentEdge = self._net.addEdge(attrs['id'],
                    attrs['from'], attrs['to'], prio, function, name)
                if attrs.has_key('shape'):
                    self.processShape(self._currentEdge, attrs['shape'])
            else:
                if function in ['crossing', 'walkingarea']:
                    self._net._crossings_and_walkingAreas.add(attrs['id'])
                self._currentEdge = None
        if name == 'lane' and self._currentEdge!=None:
            self._currentLane = self._net.addLane(
                    self._currentEdge, 
                    float(attrs['speed']),
                    float(attrs['length']),
                    attrs.get('allow'),
                    attrs.get('disallow'))
            if attrs.has_key('shape'):
                self._currentShape = attrs['shape'] # deprecated: at some time, this is mandatory
            else:
                self._currentShape = ""
        if name == 'junction':
            if attrs['id'][0]!=':':
                self._currentNode = self._net.addNode(attrs['id'], attrs['type'], (float(attrs['x']), float(attrs['y'])), attrs['incLanes'].split(" ") )
        if name == 'succ' and self._withConnections: # deprecated
            if attrs['edge'][0]!=':':
                self._currentEdge = self._net.getEdge(attrs['edge'])
                self._currentLane = attrs['lane']
                self._currentLane = int(self._currentLane[self._currentLane.rfind('_')+1:])
            else:
                self._currentEdge = None
        if name == 'succlane' and self._withConnections: # deprecated
            lid = attrs['lane']
            if lid[0]!=':' and lid!="SUMO_NO_DESTINATION" and self._currentEdge:
                connected = self._net.getEdge(lid[:lid.rfind('_')])
                tolane = int(lid[lid.rfind('_')+1:])
                if attrs.has_key('tl') and attrs['tl']!="":
                    tl = attrs['tl']
                    tllink = int(attrs['linkIdx'])
                    tlid = attrs['tl']
                    toEdge = self._net.getEdge(lid[:lid.rfind('_')])
                    tolane2 = toEdge._lanes[tolane]
                    tls = self._net.addTLS(tlid, self._currentEdge._lanes[self._currentLane], tolane2, tllink)
                    self._currentEdge.setTLS(tls)
                else:
                    tl = ""
                    tllink = -1
                toEdge = self._net.getEdge(lid[:lid.rfind('_')])
                tolane = toEdge._lanes[tolane]
                self._net.addConnection(self._currentEdge, connected, self._currentEdge._lanes[self._currentLane], tolane, attrs['dir'], tl, tllink)
        if name == 'connection' and self._withConnections and attrs['from'][0] != ":":
            fromEdgeID = attrs['from']
            toEdgeID = attrs['to']
            if not (fromEdgeID in self._net._crossings_and_walkingAreas or toEdgeID in
                    self._net._crossings_and_walkingAreas):
                fromEdge = self._net.getEdge(fromEdgeID)
                toEdge = self._net.getEdge(toEdgeID)
                fromLane = fromEdge.getLane(int(attrs['fromLane']))
                toLane = toEdge.getLane(int(attrs['toLane']))
                if attrs.has_key('tl') and attrs['tl']!="":
                    tl = attrs['tl']
                    tllink = int(attrs['linkIndex'])
                    tls = self._net.addTLS(tl, fromLane, toLane, tllink)
                    fromEdge.setTLS(tls)
                else:
                    tl = ""
                    tllink = -1
                self._net.addConnection(fromEdge, toEdge, fromLane, toLane, attrs['dir'], tl, tllink)
        if self._withFoes and name=='ROWLogic': # 'row-logic' is deprecated!!!
            self._currentNode = attrs['id']
        if name == 'logicitem' and self._withFoes: # deprecated
            self._net.setFoes(self._currentNode, int(attrs['request']), attrs["foes"], attrs["response"])
        if name == 'request' and self._withFoes:
            self._currentNode.setFoes(int(attrs['index']), attrs["foes"], attrs["response"])
        if self._withPhases and name=='tlLogic': # tl-logic is deprecated!!!
            self._currentProgram = self._net.addTLSProgram(attrs['id'], attrs['programID'], int(attrs['offset']), attrs['type'])
        if self._withPhases and name=='phase':
            self._currentProgram.addPhase(attrs['state'], int(attrs['duration']))
        if name == 'roundabout':
            self._net.addRoundabout(attrs['nodes'].split())
        if name == 'param':
            if self._currentLane!=None:
                self._currentLane._params[attrs['key']] = attrs['value']

    def characters(self, content):
        if self._currentLane!=None:
            self._currentShape = self._currentShape + content


    def endElement(self, name):
        if name == 'lane':
            if  self._currentLane:
                self.processShape(self._currentLane, self._currentShape)
                self._currentShape = ""
            self._currentLane = None
        if name == 'edge':
            if self._currentEdge and self._currentEdge._shape is None:
                self._currentEdge.rebuildShape();
            self._currentEdge = None
        if name=='ROWLogic' or name=='row-logic': # 'row-logic' is deprecated!!!
            self._haveROWLogic = False
        if self._withPhases and (name=='tlLogic' or name=='tl-logic'): # tl-logic is deprecated!!!
            self._currentProgram = None

    def processShape(self, object, shapeString):
        cshape = []
        es = shapeString.rstrip().split(" ")
        for e in es:
            p = e.split(",")
            cshape.append((float(p[0]), float(p[1])))
        object.setShape(cshape)


    def getNet(self):
        return self._net


def readNet(filename, **others):
    netreader = NetReader(**others)
    try:
        if not os.path.isfile(filename):
            print("Network file '%s' not found" % filename, file=sys.stderr)
            sys.exit(1)
        parse(filename, netreader)
    except None:
        print("Please mind that the network format has changed in 0.13.0, you may need to update your network!", file=sys.stderr)
        sys.exit(1)
    return netreader.getNet()


''' define vehicle class for rush hour project '''

class Vehicle:
    def __init__(self, id, depart):
        self._id = id
        self._depart = depart
        self._newdepart = 0
        self._route = []
        self._timewindow = []
        self._timewindow_duration = 0
        self._time_to_bottleneck = 0
        self._distance_to_bottleneck = 0

    def setRoute(self, route):
        self._route = route
    def setTimetoBottleneck(self, time_to_bottleneck):
        self._time_to_bottleneck = time_to_bottleneck
    def setDistancetoBottleneck(self, distance_to_bottleneck):
        self._distance_to_bottleneck = distance_to_bottleneck
    def setTimeWindow(self, duration):
        self._timewindow_duration = duration
        start = self._time_to_bottleneck+self._depart
        end = start + self._timewindow_duration
        self._timewindow.append(start)
        self._timewindow.append(end)
    def setNewDepart(self, newdeaprt):
        self._newdepart = newdeaprt

    def getRoute(self):
        return self._route
    def getTimeWindow(self):
        return self._timewindow
    def getID(self):
        return self._id
    def getTimetoBottleneck(self):
        return self._time_to_bottleneck

      

    def __repr__(self):
        return '<vehicle id="%s" depart="%s" time_to_bottleneck="%s" window_start="%s" window_end="%s"/>\n' % (self._id, self._depart, self._time_to_bottleneck, self._timewindow[0], self._timewindow[1])


class VehicleList:
    def __init__(self, net):
        self._vehicles = []
        self._net  = net
        self._id2vehicle = {}
        self._bottlenecks = []
        
    def addVehicle(self, id, depart):
        if id not in self._id2vehicle:
            v = Vehicle(id, depart)
            self._vehicles.append(v)
            self._id2vehicle[id] = v
        return self._id2vehicle[id]

    def addBottlenecks(self, bottlenecksString):
        cbottlenecks = []
        cbottlenecks = bottlenecksString.rstrip().split(" ")
        for c in cbottlenecks:
            self._bottlenecks.append(self._net._id2edge[c])
        for v in self._vehicles:
            time_to_bottleneck, distance_to_bottleneck = self.calcTimeDistance(v.getRoute(), self._bottlenecks[0])
            v.setTimetoBottleneck(time_to_bottleneck)
            v.setDistancetoBottleneck(distance_to_bottleneck)

    def genRandomDuration(self, unit, window_range):
        window = [15, 11, 15, 5, 8, 14, 11, 9, 9, 14, 15, 15, 11, 7, 11, 15, 7, 13, 5, 14, 10, 10, 6, 8, 12, 5, 6, 13, 9, 12, 14, 11, 13, 13, 7, 7, 8, 9, 10, 5, 12, 11, 14, 14, 9, 6, 7, 11, 14, 13, 14, 7, 7, 14, 5, 5, 14, 11, 11, 13, 14, 15, 9, 10, 11, 6, 10, 9, 6, 9, 15, 14, 5, 10, 10, 9, 10, 11, 14, 10, 15, 7, 15, 10, 9, 11, 8, 10, 8, 6, 14, 7, 5, 9, 8, 14, 9, 8, 12, 5, 10, 15, 7, 10, 8, 8, 12, 5, 7, 12, 7, 5, 12, 5, 10, 8, 9, 12, 7, 9, 13, 12, 13, 11, 12, 14, 6, 9, 5, 8, 11, 13, 7, 8, 6, 11, 15, 6, 5, 13, 8, 5, 13, 14, 10, 7, 5, 13, 8, 10, 15, 5, 12, 15, 10, 9, 11, 8, 10, 9, 12, 7, 10, 8, 6, 8, 9, 5, 5, 10, 11, 15, 5, 8, 14, 6, 6, 12, 11, 5, 5, 13, 15, 14, 8, 11, 8, 10, 14, 8, 15, 6, 11, 5, 7, 8, 14, 14, 7, 14, 6, 12, 10, 9, 13, 15, 8, 11, 7, 14, 5, 13, 6, 11, 7, 14, 8, 8, 13, 6, 10, 9, 6, 15, 6, 8, 13, 9, 10, 10, 15, 13, 15, 15, 11, 7, 10, 11, 14, 9, 8, 13, 7, 12, 11, 5, 11, 6, 9, 15, 10, 6, 9, 13, 11, 12, 5, 8, 7, 7, 9, 14, 12, 8, 12, 15, 13, 13, 6, 15, 11, 15, 6, 10, 12, 7, 14, 8, 10, 8, 6, 10, 10, 7, 13, 5, 13, 11, 13, 8, 8, 15, 15, 14, 14, 5, 12, 12, 6, 9, 12, 12, 5, 10, 15]
        #window = []
        for i,v in enumerate(self._vehicles):
            #v.setTimeWindow(int(random.expovariate(rate)))
            #window.append(random.randrange(window_range[0], window_range[1]+1))
            #v.setTimeWindow(window[i]*unit*2)
            v.setTimeWindow(window[i]*unit*2)
        #print(window)

    def calcTimeDistance(self, route, bottleneck):
        time = 0
        distance = 0
        for r in route:
            if r.getID() != bottleneck.getID():
                time += r.getLength()/r.getSpeed()
                distance += r.getLength()
            else:
                break
        return int(time), int(distance)

    def getVehicles(self):
        return self._vehicles

    def getVehicle(self, id):
        return self._id2vehicle[id]


    def __repr__(self):
        vlist = ''
        for v in self._vehicles:
            vlist += repr(v)
            for r in v._route:
                vlist += (str(r._id)+' ')
            vlist += '\n'
        return vlist[:-1]

class Scheduler:
    def __init__(self, unit, capacity, vehiclelist):
        self._unit = unit
        self._rushhour_offset = 0
        self._rushhour_period = 0
        self._vehiclelist =  vehiclelist
        self._id2window = {}
        self._window_list = self.setWindowList()
        #for CSP solver 
        self._problem = None
        self._rushhour_capacity = capacity
        self._solution = {}
        self._sorted_solution = None

    #def setRushHourCapacity(self, capacity):
     #   self._rushhour_capacity = capacity

    def setWindowList(self):
        window_list = []
        vehicles = self._vehiclelist.getVehicles()
        for v in vehicles:
            window = v.getTimeWindow()
            window = [int(w/self._unit) for w in window]
            window_list.append(window)
            
        window_list = np.array(window_list)
        self.shiftOffset(window_list)

        for i,v in enumerate(vehicles):
            self._id2window[v.getID()] = window_list[i,:]
            #print(window_list[i,:])
    
        return window_list
    
    def shiftOffset(self, windowlist):
        self._rushhour_offset = np.min(windowlist)
        self._rushhour_period = np.max(windowlist) - self._rushhour_offset + 1 
        windowlist -= self._rushhour_offset

    def calcNewDepart(self):
        self._sorted_solution = self._solution.copy()
        for v in self._vehiclelist.getVehicles():
            newdeaprt = (self._solution[v.getID()]+ self._rushhour_offset)*self._unit - v.getTimetoBottleneck()
            if newdeaprt < 0:
                newdeaprt = 0
            self._sorted_solution[v.getID()] = newdeaprt
            v.setNewDepart(newdeaprt)
        self._sorted_solution = sorted(self._sorted_solution.items(), key=operator.itemgetter(1))
        return self._sorted_solution

    def calcExNewDepart(self, option):
        self._sorted_solution = {}
        histogram = np.zeros(self._rushhour_period)
        for v in self._vehiclelist.getVehicles():
            window = self._id2window[v.getID()]
            if option == 1:
                time_slot = random.randrange(window[0], window[1]+1)
            else:
                time_slot = window[1]
            histogram[time_slot] += 1
            newdeaprt = (time_slot+ self._rushhour_offset)*self._unit - v.getTimetoBottleneck()
            if newdeaprt < 0:
                newdeaprt = 0
            self._sorted_solution[v.getID()] = newdeaprt
            v.setNewDepart(newdeaprt)

        print("current capacity of slots (random or last):")
        print(histogram)
        self._sorted_solution = sorted(self._sorted_solution.items(), key=operator.itemgetter(1))
        return self._sorted_solution

    def capacityConstraint(self, time_index, capacity, *variables):
        count = 0
        for v in variables:
            if v == time_index:
                count += 1
        return count <= capacity

    # def __call__(self):
    #     vehicles = self._vehiclelist.getVehicles()[:]
    #     capacity = np.ones(self._rushhour_period)*self._rushhour_capacity
    #     for step in range(1): 
    #         self._problem = Problem(MinConflictsSolver(5))
    #         self._problem.addVariables([v.getID() for v in vehicles], range(self._rushhour_period))
    #         for v in vehicles:
    #             window = self._id2window[v.getID()]
    #             self._problem.addConstraint(InSetConstraint(range(window[0], window[1]+1)), [v.getID()])
    #         for i in range(self._rushhour_period):
    #             self._problem.addConstraint(FunctionConstraintRushHour(self.capacityConstraint, i, capacity[i]-step*2))

    #         ret = self._problem.getMinConflictSolution(self._solution.copy())
    #         if ret == None:
    #             print('no solution')
    #             break
    #         else:
    #             self._solution = ret
    #             print(self._solution)
        
    #     delay = []
    #     histogram_after = np.zeros(self._rushhour_period)
    #     histogram_before = np.zeros(self._rushhour_period)
    #     hiswin = [[] for i in range(self._rushhour_period)]
    #     for v in vehicles:
    #         window = self._id2window[v.getID()]
    #         delay.append(self._solution[v.getID()]-window[0]+1)
    #         histogram_before[window[0]] += 1
    #         histogram_after[self._solution[v.getID()]] += 1
    #         hiswin[self._solution[v.getID()]].append([window[0],window[1]])
    #     delay = np.array(delay)
    #     print("average starting time slot = " + str(np.mean(delay)))
    #     print("previous capacity of slots:")
    #     print(histogram_before)
    #     print("current capacity of slots:")
    #     print(histogram_after)
    #     #print(hiswin)

    #     return self.calcNewDepart()
    def __call__(self):
        vehicles = self._vehiclelist.getVehicles()[:]
        capacity = np.ones(self._rushhour_period)*self._rushhour_capacity
        
        for step in range(4): 
            
            self._problem = Problem(MinConflictsSolver(100))
            self._problem.addVariables([v.getID() for v in vehicles], range(self._rushhour_period))
            

            for v in vehicles:
                window = self._id2window[v.getID()]
                self._problem.addConstraint(InSetConstraint(range(window[0], window[1]+1)), [v.getID()])
            for i in range(self._rushhour_period):
                self._problem.addConstraint(FunctionConstraintRushHour(self.capacityConstraint, i, capacity[i]))

            ret = self._problem.getMinConflictSolution()
            if ret == None:
                print('no solution')
                break
            else:
                for r in ret.keys():
                    self._solution[r] = ret[r]
                print(self._solution)
            
            count_capacity = np.zeros(self._rushhour_period)
            countwin = [{} for i in range(self._rushhour_period)]
            for v in self._vehiclelist.getVehicles():
                count_capacity[self._solution[v.getID()]] += 1
                countwin[self._solution[v.getID()]][v.getID()] = window[1]-window[0]+1
            print(count_capacity)
            capacity = self._rushhour_capacity- (step+1)*2 - count_capacity
            print(capacity)
            if sum(capacity) < 0:
                break
            vehicles = []
            for i in range(self._rushhour_period):
                if capacity[i] < 0:
                    sorted_countwin= sorted(countwin[i].items(), key=operator.itemgetter(1))
                    sorted_countwin.reverse()
                    for j in range(abs(int(capacity[i]))):
                        vehicles.append(self._vehiclelist.getVehicle(sorted_countwin[j][0]))
                    capacity[i] = 0
            
        
        delay = []
        histogram_after = np.zeros(self._rushhour_period)
        histogram_before = np.zeros(self._rushhour_period)
        hiswin = [[] for i in range(self._rushhour_period)]
        dist = np.zeros(self._rushhour_period)
        for v in self._vehiclelist.getVehicles():
            window = self._id2window[v.getID()]
            delay.append(self._solution[v.getID()]-window[0]+1)
            for i in range(window[1]-window[0]+1):
                dist[window[0]+i] += 1
            histogram_before[window[0]] += 1
            histogram_after[self._solution[v.getID()]] += 1
            hiswin[self._solution[v.getID()]].append([window[0],window[1]])
        delay = np.array(delay)
        print("average starting time slot = " + str(np.mean(delay)))
        print("previous capacity of slots:")
        print(histogram_before)
        print("current capacity of slots:")
        print(histogram_after)
        print("vehicle distribution over time slots:")
        print(dist)

        return self.calcNewDepart()

    def getSortedSolution(self):
        return self._sorted_solution
    def getVehicleList(self):
        return self._vehiclelist
   

class VehicleListReader(handler.ContentHandler):

    def __init__(self, net, **others):
        self._vehiclelist = others.get('vehiclelist', VehicleList(net))
        self._currentVehicle = None
        self._net = net

    def startElement(self, name, attrs):
        if name == 'vehicle':
            self._currentVehicle =  self._vehiclelist.addVehicle(attrs['id'], float(attrs['depart']))
        if name == 'route' and  self._currentVehicle != None:
            self.processRoute(attrs['edges'])
            self._currentVehicle = None
    
    def processRoute(self, routeString):
        croute = []
        route = []
        croute = routeString.rstrip().split(" ")
        for c in croute:
            route.append(self._net._id2edge[c])
        self._currentVehicle.setRoute(route)

    def getVehicleList(self):
        return self._vehiclelist


def readVehicleList(filename, net, **others):
    vehiclelistreader = VehicleListReader(net, **others)
    try:
        if not os.path.isfile(filename):
            print("Network file '%s' not found" % filename, file=sys.stderr)
            sys.exit(1)
        parse(filename, vehiclelistreader)
    except None:
        print("Please mind that the network format has changed in 0.13.0, you may need to update your network!", file=sys.stderr)
        sys.exit(1)

    return vehiclelistreader.getVehicleList()

def generateRouteFile(filename, scheduler):
    routes_xml = ET.Element("routes", noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd", xsi="http://www.w3.org/2001/XMLSchema-instance")
    
    solution = scheduler.getSortedSolution()
    vehiclelist = scheduler.getVehicleList()
    for s in solution:
        vehicle = vehiclelist.getVehicle(s[0])
        depart = s[1]
        vehicle_xml = ET.SubElement(routes_xml, "vehicle", depart="%s.00" % depart, id="%s" % s[0])
        routes = ""
        for r in vehicle.getRoute():
            routes += str(r.getID())
            routes += " "
        ET.SubElement(vehicle_xml, "route", edges=routes)
    
    tree = ET.ElementTree(routes_xml)
    tree.write(filename) 

''' end rush hour '''

