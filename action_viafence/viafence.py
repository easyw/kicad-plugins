#!/usr/bin/env python2
import math
#import pyclipper
from bisect import bisect_left
import wx
import pcbnew

showHelpers = True #False #True

def verbose(object, *args, **kwargs):
    global verboseFunc
    verboseFunc(object, *args, **kwargs)

# Returns the slope of a line
def getLineSlope(line):
    return math.atan2(line[0][1]-line[1][1], line[0][0]-line[1][0])

# Returns the length of a line
def getLineLength(line):
    return math.hypot(line[0][0]-line[1][0], line[0][1]-line[1][1])

# Returns a sub path in a path with a path specification (startIdx, stopIdx)
def getSubPath(path, pathSpec):
    listModulus = len(path)
    if (pathSpec[1] < pathSpec[0]): pathSpec[1] += listModulus
    return [path[i % listModulus] for i in range(pathSpec[0], pathSpec[1]+1)]

# Returns a list of subpaths with a list of path specifications
def getSubPaths(path, pathSpecList):
    return [getSubPath(path, pathSpec) for pathSpec in pathSpecList if (pathSpec[0] != pathSpec[1])]

# Splits a path using a list of indices representing points on the path
def splitPathByPoints(path, splitList):
    pathSpecList = [[splitList[item], splitList[item+1]] for item in range(0, len(splitList)-1)]
    return getSubPaths(path, pathSpecList)

# Splits a path around a list of list of indices representing a subpath within the original path
def splitPathByPaths(path, splitList):
    pathSpecList = [[splitList[item][-1], splitList[(item+1)%len(splitList)][0]] for item in range(0, len(splitList))]
    return getSubPaths(path, pathSpecList)

# Return a cumulative distance vector representing the distance travelled along
# the path at each path vertex
def getPathCumDist(path):
    cumDist = [0.0]
    for vertexId in range(1, len(path)):
        cumDist += [cumDist[-1] + getLineLength([path[vertexId], path[vertexId-1]])]

    return cumDist

# Return a list of all vertex indices where the angle between
# the two lines connected to the vertex deviate from a straight
# path more by the tolerance angle in degrees
# This function is used to find bends that are larger than a certain angle
def getPathVertices(path, angleTolerance):
    angleTolerance = angleTolerance * math.pi / 180
    vertices = []

    # Look through all vertices except start and end vertex
    # Calculate by how much the lines before and after the vertex
    # deviate from a straight path.
    # If the deviation angle exceeds the specification, store it
    for vertexIdx in range(1, len(path)-1):
        prevSlope = getLineSlope([path[vertexIdx+1], path[vertexIdx]])
        nextSlope = getLineSlope([path[vertexIdx-1], path[vertexIdx]])
        deviationAngle = abs(prevSlope - nextSlope) - math.pi
        if (abs(deviationAngle) > angleTolerance):
            vertices += [vertexIdx]

    return vertices

# Uses the cross product to check if a point is on a line defined by two other points
def isPointOnLine(point, line):
    cross = (line[1][1] - point[1]) * (line[0][0] - point[0]) - (line[1][0] - point[0]) * (line[0][1] - point[1])

    if  (   ((line[0][0] <= point[0] <= line[1][0]) or (line[1][0] <= point[0] <= line[0][0]))
        and ((line[0][1] <= point[1] <= line[1][1]) or (line[1][1] <= point[1] <= line[0][1]))
        and (cross == 0) ):
        return True
    return False

# Returns a list of path indices touching any item in a list of points
def getPathsThroughPoints(path, pointList):
    touchingPaths = []

    for vertexIdx in range(0, len(path)):
        fromIdx = vertexIdx
        toIdx = (vertexIdx+1) % len(path)

        # If a point in the pointList is located on this line, store the line
        for point in pointList:
            if isPointOnLine(point, [ path[fromIdx], path[toIdx] ]):
                touchingPaths += [[fromIdx, toIdx]]
                break

    return touchingPaths

# A small linear interpolation class so we don't rely on scipy or numpy here
class LinearInterpolator(object):
    def __init__(self, x_list, y_list):
        self.x_list, self.y_list = x_list, y_list
        intervals = zip(x_list, x_list[1:], y_list, y_list[1:])
        self.slopes = [(y2 - y1)/(x2 - x1) for x1, x2, y1, y2 in intervals]
    def __call__(self, x):
        i = bisect_left(self.x_list, x) - 1
        return self.y_list[i] + self.slopes[i] * (x - self.x_list[i])

# Interpolate a path with (x,y) vertices using a third parameter t
class PathInterpolator:
    def __init__(self, t, path):
        # Quick and dirty transpose path so we get two list with x and y coords
        # And set up two separate interpolators for them
        x = [vertex[0] for vertex in path]
        y = [vertex[1] for vertex in path]
        self.xInterp = LinearInterpolator(t, x)
        self.yInterp = LinearInterpolator(t, y)
    def __call__(self, t):
        # Return interpolated coordinates on the original path
        return [self.xInterp(t), self.yInterp(t)]

# A small pyclipper wrapper class to expand a line to a polygon with a given offset
def expandPathsToPolygons(pathList, offset):
    import pyclipper
    # Use PyclipperOffset to generate polygons that surround the original
    # paths with a constant offset all around
    co = pyclipper.PyclipperOffset()
    for path in pathList: co.AddPath(path, pyclipper.JT_ROUND, pyclipper.ET_OPENROUND)
    return co.Execute(offset)

# A small pyclipper wrapper to trim parts of a polygon using another polygon
def clipPolygonWithPolygons(path, clipPathList):
    import pyclipper
    pc = pyclipper.Pyclipper()
    pc.AddPath(path, pyclipper.PT_SUBJECT, True)
    for clipPath in clipPathList: pc.AddPath(clipPath, pyclipper.PT_CLIP, True)
    return pc.Execute(pyclipper.CT_DIFFERENCE)

def unionPolygons(pathList):
    import pyclipper
    pc = pyclipper.Pyclipper()
    for path in pathList: pc.AddPath(path, pyclipper.PT_SUBJECT, True)
    return pc.Execute(pyclipper.CT_UNION, pyclipper.PFT_NONZERO)

def isPointInPolygon(point, path):
    import pyclipper
    return True if (pyclipper.PointInPolygon(point, path) == 1) else False

def getPathsInsidePolygon(pathList, polygon):
    filteredPathList = []

    for path in pathList:
        allVerticesInside = True
        for vertex in path:
            if not isPointInPolygon(vertex, polygon):
                allVerticesInside = False
                break
        if (allVerticesInside): filteredPathList += [path]

    return filteredPathList

# Distribute Points along a path with equal spacing to each other
# When the path length is not evenly dividable by the minimumSpacing,
# the actual spacing will be larger, but still smaller than 2*minimumSpacing
# The function does not return the start and end vertex of the path
def distributeAlongPath(path, minimumSpacing):
    # Get cumulated distance vector for the path
    # and determine the number of points that can fit to the path
    distList = getPathCumDist(path)
    nPoints = int(math.floor(distList[-1] / minimumSpacing))
    ptInterp = PathInterpolator(distList, path)
    return [ptInterp(ptIdx * distList[-1]/nPoints) for ptIdx in range(1, nPoints)]

# Find the leaf vertices in a list of paths,
# additionally it calculates the slope of the line connected to the leaf vertex
def getLeafVertices(pathList):
    allVertices = [vertex for path in pathList for vertex in path]
    leafVertices = []
    leafVertexSlopes = []

    for path in pathList:
        for vertexIdx in [0,-1]:
            if (allVertices.count(path[vertexIdx]) == 1):
                # vertex appears only once in entire path list, store away
                # Get neighbour vertex and also calculate the slope
                leafVertex = path[vertexIdx]
                neighbourVertex = path[ [1,-2][vertexIdx] ]
                leafVertices += [leafVertex]
                leafVertexSlopes += [getLineSlope([neighbourVertex, leafVertex])]

    return leafVertices, leafVertexSlopes

# Rotate and Translate a list of vertices using a given angle and offset
def transformVertices(vertexList, offset, angle):
    return [ [ round(offset[0] + math.cos(angle) * vertex[0] - math.sin(angle) * vertex[1]),
               round(offset[1] + math.sin(angle) * vertex[0] + math.cos(angle) * vertex[1]) ]
           for vertex in vertexList]

# Trims a polygon flush around the given vertices
def trimFlushPolygonAtVertices(path, vertexList, vertexSlopes, radius):
    const = 0.414
    trimPoly = [ [0, -radius], [0, 0], [0, radius], [-const*radius, radius], [-radius, const*radius],
                 [-radius, -const*radius], [-const*radius, -radius] ]
    trimPolys = [transformVertices(trimPoly, vertexPos, vertexSlope)
        for vertexPos, vertexSlope in zip(vertexList, vertexSlopes)]

    trimPolys = unionPolygons(trimPolys)

    verbose(trimPolys, isPolygons=True)

    return clipPolygonWithPolygons(path, trimPolys)

######################

def create_Text(txt, p, w, lyr):
    pcb = pcbnew.GetBoard()
    mytxt = pcbnew.TEXTE_PCB(pcb)
    mytxt.SetText(txt)
    mytxt.SetLayer(lyr)
    mytxt.SetPosition(p)
    mytxt.SetHorizJustify(pcbnew.GR_TEXT_HJUSTIFY_CENTER)
    mytxt.SetTextSize(pcbnew.wxSize(w,w))
    mytxt.SetThickness(int(w/4))
    pcb.Add(mytxt)
#
def distance (p1,p2):
    return math.hypot(p1[1]-p2[1],p1[0]-p2[0])
#
def generateViaFence(pathList, viaOffset, viaPitch, vFunc = lambda *args,**kwargs:None):
    global verboseFunc
    verboseFunc = vFunc
    viaPoints = []
    viaPointsTop = []
    viaPointsTopStart = []
    viaPointsBot = []
        
    # Remove zero length tracks
    pathList = [path for path in pathList if getLineLength(path) > 0]

    # Expand the paths given as a parameter into one or more polygons
    # using the offset parameter
    for offsetPoly in expandPathsToPolygons(pathList, viaOffset):
        verbose([offsetPoly], isPolygons=True)
        # Filter the input path to only include paths inside this polygon
        # Find all leaf vertices and use them to trim the expanded polygon
        # around the leaf vertices so that we get a flush, flat end
        # These butt lines are then found using the leaf vertices
        # and used to split open the polygon into multiple separate open
        # paths that envelop the original path
        localPathList = getPathsInsidePolygon(pathList, offsetPoly)
        if len(localPathList) == 0: continue # This might happen with very bad input paths

        leafVertexList, leafVertexAngles = getLeafVertices(localPathList)
        offsetPoly = trimFlushPolygonAtVertices(offsetPoly, leafVertexList, leafVertexAngles, 1.1*viaOffset)[0]
        buttLineIdxList = getPathsThroughPoints(offsetPoly, leafVertexList)
        fencePaths = splitPathByPaths(offsetPoly, buttLineIdxList)

        verbose([offsetPoly], isPolygons=True)
        verbose([leafVertexList], isPoints=True)
        verbose(fencePaths, isPaths=True)

        # With the now separated open paths we perform via placement on each one of them
        i = 0
        j = 0
        k = 0
        for fencePath in fencePaths:
            # For a nice via fence placement, we identify vertices that differ from a straight
            # line by more than 10 degrees so we find all non-arc edges
            # We combine these points with the start and end point of the path and use
            # them to place fixed vias on their positions
            tolerance_degree = 10
            fixPointIdxList = [0] + getPathVertices(fencePath, tolerance_degree) + [-1]
            #fixPointList = [fencePath[idx] for idx in fixPointIdxList]
            fixPointList = []
            #wx.LogMessage(str(fixPointIdxList))
            for idx in fixPointIdxList:
                fixPointList.append(fencePath[idx])
                #wx.LogMessage(str(fencePath[idx]))
                #wx.LogMessage('fixP'+str(fencePath[idx][0])+':'+str(fencePath[idx][1]))
                #create_Text(str(i),pcbnew.wxPoint(fencePath[idx][0],fencePath[idx][1]),pcbnew.FromMM(1.0),pcbnew.F_SilkS)
                ##create_Text(str(k),pcbnew.wxPoint(fencePath[idx][0],fencePath[idx][1]),pcbnew.FromMM(1.4),pcbnew.Eco2_User)
                ### questi if k == 0:
                ###     viaPointsTopStart.append(fencePath[idx])
                #elif k == 1:
                #    viaPointsBot.append(fencePath[idx])
                i+=1
            verbose(fixPointList, isPoints=True)

            viaPoints += fixPointList
            # Then we autoplace vias between the fixed via locations by satisfying the
            # minimum via pitch given by the user
            subPathCounter = 0
            for subPath in splitPathByPoints(fencePath, fixPointIdxList):
                #wx.LogMessage('subPathCounter: '+str(subPathCounter))
                p1 = fencePath[fixPointIdxList[subPathCounter]]
                if k == 0:
                    #viaPointsTop.append(p1)
                    if len (viaPointsTop)>0:
                        #wx.LogMessage(str(viaPointsTop[-1])+'::'+str(p1))
                        #wx.LogMessage(str(p))
                        if viaPointsTop[-1][0] != p1[0] and viaPointsTop[-1][1] != p1[1]:
                            viaPointsTop.append(p1)
                    else:
                        viaPointsTop.append(p1)
                elif k == 1:
                    #viaPointsTop.append(p1)
                    if len (viaPointsBot)>0:
                        #wx.LogMessage(str(viaPointsBot[-1])+'::'+str(p1))
                        #wx.LogMessage(str(p))
                        if viaPointsBot[-1][0] != p1[0] and viaPointsBot[-1][1] != p1[1]:
                            viaPointsBot.append(p1)
                    else:
                        viaPointsBot.append(p1)
                subPathCounter+=1
                distributedPoints = distributeAlongPath(subPath, viaPitch)
                viaPoints += distributedPoints #distributeAlongPath(subPath, viaPitch)
                #wx.LogMessage(str(len(distributedPoints)))
                for p in (distributedPoints):
                    #create_Text(str(j),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.4),pcbnew.B_SilkS)
                    j+=1
                    #create_Text(str(i),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.4),pcbnew.Eco2_User)
                    ##create_Text(str(k),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.4),pcbnew.Eco2_User)
                    if k == 0:
                        viaPointsTop.append(p)
                        if len (viaPointsTop)>0:
                            #wx.LogMessage(str(viaPointsTop[-1])+'::'+str(p))
                            #wx.LogMessage(str(p))
                            if viaPointsTop[-1][0] != p[0] and viaPointsTop[-1][1] != p[1]:
                                viaPointsTop.append(p)
                        else:
                            viaPointsTop.append(p)
                    elif k == 1:
                        viaPointsBot.append(p)
                        if len (viaPointsBot)>0:
                            #wx.LogMessage(str(viaPointsBot[-1])+'::'+str(p))
                            #wx.LogMessage(str(p))
                            if viaPointsBot[-1][0] != p[0] and viaPointsBot[-1][1] != p[1]:
                                viaPointsBot.append(p)
                        else:
                            viaPointsBot.append(p)
                    
                    p3 = fencePath[fixPointIdxList[subPathCounter]]
                    if k == 0:
                        viaPointsTop.append(p3)
                    elif k ==1:
                        viaPointsBot.append(p3)
                    #elif k == 1:
                    #    viaPointsBot.append(p)
                #j+=1
                #wx.LogMessage(str(distributeAlongPath(subPath, viaPitch)))
            k+=1
            #p0Top = viaPointsTop[-1]
            #if len (viaPointsBot) > 0:
            #    p0Bot = viaPointsBot[-1]
    ln = len (viaPointsTop)
    #wx.LogMessage(str(viaPointsTop))
    #for i,p in enumerate(viaPointsTop):
    #    if i < ln-1:
    #        if distance(p,viaPointsTop[i+1]) < viaPitch:
    #            viaPointsTop.pop(i+1)
    #            ln = len (viaPointsTop)
    #    wx.LogMessage(str(p))
    #    create_Text(str(i),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.4),pcbnew.Eco2_User)
    if 0:
        filteredViaPointsTop = []
        filteredViaPointsTop.append(viaPointsTop[0])
        #for i,p in enumerate(viaPointsTop):
        i=0
        while i< len(viaPointsTop):
            j = i+1
            p = viaPointsTop[i]
            while j < len(viaPointsTop):
            #for j, p1 in enumerate(viaPointsTop[i+1:]):
                p1 = viaPointsTop[j]
                #wx.LogMessage(str(distance(p1,p))+'--'+str(viaPitch))
                if int(distance(p1,p)) > int(viaPitch):
                    #wx.LogMessage(str(p1 in filteredViaPointsTop))
                    #if str(p1) not in str(filteredViaPointsTop):
                    if (p1) not in (filteredViaPointsTop):
                        filteredViaPointsTop.append(p1)
                    else:
                        wx.LogMessage('present')
                j+=1
            i+=1
        wx.LogMessage(str(len(filteredViaPointsTop)))
        for i,p in enumerate(filteredViaPointsTop):
            create_Text(str(i),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.0),pcbnew.Eco2_User)
                    #ln = len (viaPointsTop)
        return viaPoints,filteredViaPointsTop,viaPointsBot 

    if showHelpers:
        for i,p in enumerate(viaPointsTop):
            create_Text(str(i),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.0),pcbnew.Eco2_User)
        for i,p in enumerate(viaPointsBot):
            create_Text(str(i),pcbnew.wxPoint(p[0],p[1]),pcbnew.FromMM(1.0),pcbnew.F_CrtYd)
        
    #ln = len (viaPointsBot)
    #for i,p in viaPointsBot:
    #    if i < ln-1:
    #        if distance(p,viaPointsBot[i+1]) < viaPitch:
    #            viaPointsBot.pop(i)
    #            ln = len (viaPointsBot)
    #wx.LogMessage('Top:'+str(viaPointsTop))
    #wx.LogMessage('Bot:'+str(viaPointsBot))
    return viaPoints,viaPointsTop,viaPointsBot 


