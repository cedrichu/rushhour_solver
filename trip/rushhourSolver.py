import os, sys, random, bisect, datetime, subprocess
from collections import defaultdict
import math
import optparse

SUMO_HOME = os.environ.get('SUMO_HOME',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
sys.path.append(os.path.join(SUMO_HOME, 'tools'))
import sumolib


def get_options(args=None):
    optParser = optparse.OptionParser()
    optParser.add_option("-r", "--route-file", dest="routefile",
                            help="define the route file (mandatory)")
    optParser.add_option("-n", "--net-file", dest="netfile",
                            help="define the net file (mandatory)")
    optParser.add_option("-o", "--output-file", dest="outputfile",
                            help="output sorted routing files")
    optParser.add_option("--last", action="store_true",
                         default=False, help="run last time slot case")
    optParser.add_option("--rand", action="store_true",
                         default=False, help="run random time slot case")
    (options, args) = optParser.parse_args(args=args)
    return options

def main(options):
    window_range = [5,15]
    unit = 30
    capacity_per_unit = 15

    net = sumolib.net.readNet(options.netfile)
    vehiclelist = sumolib.net.readVehicleList(options.routefile, net)
    vehiclelist.addBottlenecks('11950069')
    vehiclelist.genRandomDuration(unit,window_range)
  
    scheduler = sumolib.net.Scheduler(unit, capacity_per_unit, vehiclelist)
    print scheduler()
    sumolib.net.generateRouteFile(options.outputfile, scheduler)
    
    if options.rand:
        scheduler.calcExNewDepart(1)
        index = options.outputfile.find('.xml')
        outputfile = options.outputfile[:index]+'_random'+ options.outputfile[index:]
        sumolib.net.generateRouteFile(outputfile, scheduler)

    if options.last:
        scheduler.calcExNewDepart(2)
        index = options.outputfile.find('.xml')
        outputfile = options.outputfile[:index]+'_last'+ options.outputfile[index:]
        sumolib.net.generateRouteFile(outputfile, scheduler)


    

if __name__ == "__main__":
    main(get_options())


