#!/usr/bin/env python3
#
#   planarutils.py
#
#   Provide several utilities for planar geometry.  In particular:
#
#     PointNearPoint(dist, pointA, pointB)
#
#     PointNearSegment(dist, point, segment)
#     SegmentCrossSegment(segmentA, segmentB)
#     SegmentNearSegment(dist, segmentA, segmentB)
#
#     PointInTriangle(point, triangle)
#     PointNearTriangle(dist, point, triangle)
#     SegmentCrossTriangle(segment, triangle)
#
#     ValidBox(box)                 Test convex box.
#     PointInBox(point, box)
#     PointNearBox(dist, point, box)
#     SegmentCrossBox(segment, box)
#
#     SegmentCrossArc(segment, arc)
#
#   where
#     distance d                    is a distance of proximity
#     point    p = (x,y)            is a point at x/y coordinates
#     segment  s = (p1, p2)         is the line segment between the points
#     triangle t = (p1, p2, p3)     is the triangle with given corners
#     box      b = (p1, p2, p3, p4) is the box with given corners
#     arc      a = (pc, pa, pb)     is the arc about center pc from pa to pb
#
#   All functions return True or False.
#
import numpy as np
######################################################################
#
#   RELATIVE TO POINT
#
#   Proximity of Point to Point
#
def PointNearPoint(d, pA, pB):
    return ((pA[0]-pB[0])**2 + (pA[1]-pB[1])**2 <= d**2)


######################################################################
#
#   RELATIVE TO SEGMENT
#
#   Proximity of Point to Segment
#
def PointNearSegment(d, p, s):
    # Precompute the relative vectors.
    (vx, vy) = (s[1][0]-s[0][0], s[1][1]-s[0][1])
    (rx, ry) = (   p[0]-s[0][0],    p[1]-s[0][1])

    # Precompute the vector products.
    rTv = rx*vx + ry*vy
    rTr = rx**2 + ry**2
    vTv = vx**2 + vy**2

    # Check the point-to-point distance when outside the segment range.
    if (rTv <= 0):
        return (rTr <= d**2)
    if (rTv >= vTv):
        return (rTr - 2*rTv + vTv <= d**2)

    # Check the orthogonal point-to-line distance inside the segment range.
    return ((rx*vy - ry*vx)**2 <= vTv * d**2)

#
#   Segment Crossing Segment
#
def SegmentCrossSegment(sA, sB):
    # Precompute the relative vectors.
    (ux, uy) = (sA[1][0]-sA[0][0], sA[1][1]-sA[0][1])
    (vx, vy) = (sB[1][0]-sB[0][0], sB[1][1]-sB[0][1])
    (rx, ry) = (sB[0][0]-sA[0][0], sB[0][1]-sA[0][1])

    # Precompute the vector cross products.
    uXv = ux*vy - uy*vx
    rXu = rx*uy - ry*ux
    rXv = rx*vy - ry*vx

    # Check the intersection.
    if (uXv > 0):
        return ((rXu > 0) and (rXu < uXv) and (rXv > 0) and (rXv < uXv))
    else:
        return ((rXu < 0) and (rXu > uXv) and (rXv < 0) and (rXv > uXv))


#
#   Segment Crossing Segment
# Ref: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
#
def WhereSegmentCrossSegment(sA, sB):
    # precompute parameters
    p = sA[0]
    r = (sA[1][0] - sA[0][0], sA[1][1] - sA[0][1])

    q = sB[0]
    s = (sB[1][0] - sB[0][0], sB[1][1] - sB[0][1])

    ## Compute intersection cases
    # (q - p) x r
    num = (q[0] - p[0], q[1] - p[1])
    num = num[0]*r[1] - num[1]*r[0]
    # (r x s)
    denom = r[0]*s[1] - r[1]*s[0]

    ## Cases
    if (num == 0 and denom == 0):
        # two lines are colinear
        t0 = (q[0] - p[0])*r[0] + (q[1] - p[1])*r[1]
        t0 /= r[0]**2 + r[1]**2

        t1 = t0 + (s[0]*r[0] + s[1]*r[1]) / (r[0]**2 + r[1]**2)

        # check if [t0, t1] intersects [0, 1]
        if (t0 < 0 and t1 < 0):
            # no intersection
            return -1
        elif (t0 > 1 and t1 > 1):
            # no intersection
            return -1

        # there is an intersection, return the two endpoints
        if (t0 < 0):
            int1 = p
            int2 = (p[0] + t1*r[0], p[1] + t1*r[1])
            return (int1, int2)
        elif (t0 > 1):
            int1 = (p[0] + t1*r[0], p[1] + t1*r[1])
            int2 = (p[0] + r[0], p[1] + r[1])
            return (int1, int2)
        elif (t1 < 0):
            int1 = p
            int2 = (p[0] + t0*r[0], p[1] + t0*r[1])
            return (int1, int2)
        elif (t1 > 1):
            int1 = (p[0] + t0*r[0], p[1] + t0*r[1])
            int2 = (p[0] + r[0], p[1] + r[1])
            return (int1, int2)
        else:
            # both t0 and t1 are in interval
            int1 = (p[0] + t0*r[0], p[1] + t0*r[1])
            int2 = (p[0] + t1*r[0], p[1] + t1*r[1])
            return (int1, int2)

    if (denom == 0 and num != 0):
        # no intersection
        return -1

    # now we can safely calculate u
    u = num / denom
    if (denom != 0 and u >= 0 and u <= 1):
        int = (q[0] + u*s[0], q[1] + u*s[1])
        return (int, -1)

    # if we made it here, the two line segments are not parallel and they do not intersect
    return -1


def SegmentsFromCenterline(centerline, width):
    # pull out centerline points
    pt1 = centerline[0]
    pt2 = centerline[1]

    # compute slope
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    if (dx == 0):
        # the line is vertical.
        ptA1 = (pt1[0] - width, pt1[1])
        ptA2 = (pt1[0] + width, pt1[1])

        ptB1 = (pt2[0] - width, pt2[1])
        ptB2 = (pt2[0] + width, pt2[1])

        return ((ptA1, ptA2), (ptA2, ptB2), (ptB2, ptB1), (ptB1, ptA1))

    if (dy == 0):
        # the line is horizontal.
        ptA1 = (pt1[0], pt1[1] - width)
        ptA2 = (pt1[0], pt1[1] + width)

        ptB1 = (pt2[0], pt2[1] - width)
        ptB2 = (pt2[0], pt2[1] + width)

        return ((ptA1, ptA2), (ptA2, ptB2), (ptB2, ptB1), (ptB1, ptA1))

    # line is not vertical. Get perpendicular slope
    perpslope = -dx/dy

    # compute offsets relative to pt1 or pt2
    xAoffset = np.sqrt(width*width / (1 + perpslope*perpslope))
    yAoffset = perpslope*xAoffset

    # Add the position of pt1 to get xAs
    ptA1 = (pt1[0] - xAoffset, pt1[1] - yAoffset)
    ptA2 = (pt1[0] + xAoffset, pt1[1] + yAoffset)
    # Add the position of pt1 to get xAs
    ptB1 = (pt2[0] - xAoffset, pt2[1] - yAoffset)
    ptB2 = (pt2[0] + xAoffset, pt2[1] + yAoffset)

    return ((ptA1, ptA2), (ptA2, ptB2), (ptB2, ptB1), (ptB1, ptA1))

def FinalPosition(centerline, p1, p2):
    # pull out centerline points
    pt1 = centerline[0]
    pt2 = centerline[1]

    # compute slope
    dx = pt2[0] - pt1[0]
    dy = pt2[1] - pt1[1]
    if (dx == 0):
        # the line is vertical. Look for a horizontal line
        if (abs(p1[1] - pt1[1]) > abs(p2[1] - pt1[1])):
            return [pt1[0], p1[1]]
        else:
            return [pt1[0], p2[1]]

    if (dy == 0):
        # the line is horizontal. Look for a vertical line
        if (abs(p1[0] - pt1[0]) > abs(p2[0] - pt1[0])):
            return [p1[0], pt1[1]]
        else:
            return [p2[0], pt1[1]]

    # line is not vertical. Get perpendicular slope (TODO: FIX!!)
    perpslope = -dx/dy

    # compute offsets relative to pt1 or pt2
    xAoffset = np.sqrt(width*width / (1 + perpslope*perpslope))
    yAoffset = perpslope*xAoffset

    # Add the position of pt1 to get xAs
    ptA1 = (pt1[0] - xAoffset, pt1[1] - yAoffset)
    ptA2 = (pt1[0] + xAoffset, pt1[1] + yAoffset)
    # Add the position of pt1 to get xAs
    ptB1 = (pt2[0] - xAoffset, pt2[1] - yAoffset)
    ptB2 = (pt2[0] + xAoffset, pt2[1] + yAoffset)

    return ((ptA1, ptA2), (ptA2, ptB2), (ptB2, ptB1), (ptB1, ptA1))

def PlanarCross(pt1, pt2):
    return pt1[0]*pt2[1] - pt1[1]*pt2[0]


#
#   Proximity of Segment to Rectangle
#
def SegmentCrossRectangle(s, centerline, width):
    ## Compute segments from centerline
    (srA, srB, srC, srD) = SegmentsFromCenterline(centerline, width)

    # check intersection with segments first
    iA = WhereSegmentCrossSegment(s, srA)
    iB = WhereSegmentCrossSegment(s, srB)
    iC = WhereSegmentCrossSegment(s, srC)
    iD = WhereSegmentCrossSegment(s, srD)
    ints = [iA, iB, iC, iD]

    # a line cannot intersect more than two rectangle edges
    pt1 = -1
    pt2 = -1
    possible_pt2 = -1
    for i in ints:
        if (i == -1):
            # not an intersection
            # need to check endpoints in rectangle
            continue
        elif (i[1] == -1):
            # just intersects at one point
            if (pt1 == -1):
                pt1 = i[0]
                # may need to check other endpoint later
            else:
                pt2 = i[0]
            continue
        else:
            # parallel to one of the lines! Not ideal...
            # for now, just return line endpoints (TODO: fix)
            pt1 = i[0]
            pt2 = i[1]

    if (pt2 == -1):
        # one or more endpoints may be inside of the rectangle.
        # check if other wall point has same cross product sign with rectangle edges.
        rectangle_edges = (srA, srB, srC, srD)
        for pt in s:
            lastSign = 0
            inside = False
            for edge in rectangle_edges:
                # create vector connecting to first point
                vec = (pt, s[0])
                # compute the cross product
                sign = planarCross(pt, vec)
                if (sign > 0 and lastSign < 0):
                    # point is not inside the rectangle!
                    print("No points in rectangle")
                    inside = False
                    break
                inside = True

            # add the point
            if (inside and pt1 == -1):
                pt1 = pt
            elif inside:
                pt2 = pt
                break


    # TODO: compute distance along centerline to wall (if relevant)
    if (pt2 == -1):
        # there's no wall, go to final position
        segpos = centerline[1]
    else:
        # there is a wall... compute closest position
        segpos = FinalPosition(centerline, pt1, pt2)

    return (pt1, pt2, segpos)

#
#   Proximity of Segment to Segment
#
def EndpointsNearSegmentInterior(d, sA, sB):
    # Precompute the relative vectors.
    ( vx,  vy) = (sB[1][0]-sB[0][0], sB[1][1]-sB[0][1])
    (r1x, r1y) = (sA[0][0]-sB[0][0], sA[0][1]-sB[0][1])
    (r2x, r2y) = (sA[1][0]-sB[0][0], sA[1][1]-sB[0][1])

    # Precompute the vector cross products (orthogonal distances).
    r1Xv = r1x*vy - r1y*vx
    r2Xv = r2x*vy - r2y*vx

    # If the endpoints are on opposite sides of the line, this test
    # becomes irrelevant.  Skip and report.
    if (r1Xv * r2Xv <= 0):
        return (True, False)

    # Finaly check the only closer endpoint sA[0]/sA[1] vs the segment
    # sB[0]-sB[1].  The farther endpoint can never be the shortest distance.
    vTv = vx**2 + vy**2
    if abs(r2Xv) < abs(r1Xv):
        r2Tv = r2x*vx + r2y*vy
        return (False, ((r2Tv    >=   0) and
                        (r2Tv    <= vTv) and
                        (r2Xv**2 <= vTv * d**2)))
    else:
        r1Tv = r1x*vx + r1y*vy
        return (False, ((r1Tv    >=   0) and
                        (r1Tv    <= vTv) and
                        (r1Xv**2 <= vTv * d**2)))

def SegmentNearSegment(d, sA, sB):
    # First check the endpoints-to-endpoints.
    if (PointNearPoint(d, sA[0], sB[0]) or
        PointNearPoint(d, sA[0], sB[1]) or
        PointNearPoint(d, sA[1], sB[0]) or
        PointNearPoint(d, sA[1], sB[1])):
        return True

    # Then check the endpoints to segment interiors.  This also
    # reports whether the endpoints are on opposites sides of the
    # segement.
    cross1, near = EndpointsNearSegmentInterior(d, sA, sB)
    if near:
        return True
    cross2, near = EndpointsNearSegmentInterior(d, sB, sA)
    if near:
        return True

    # If both crossed, the segments are intersecting.
    return (cross1 and cross2)


######################################################################
#
#   RELATIVE TO TRIANGLE
#
#   Proximity of Point to Triangle
#
def PointInTriangle(p, t):
    # Precompute the relative vectors.
    (apx, apy) = (t[0][0]-p[0], t[0][1]-p[1])
    (bpx, bpy) = (t[1][0]-p[0], t[1][1]-p[1])
    (cpx, cpy) = (t[2][0]-p[0], t[2][1]-p[1])

    # Precompute the cross products
    aXb = apx * bpy - apy * bpx
    bXc = bpx * cpy - bpy * cpx
    cXa = cpx * apy - cpy * apx

    # Determine whether the triangle is given CW (axb + bxc + cxa > 0) or CCW.
    if (aXb + bXc + cXa > 0):
        return (aXb >= 0) and (bXc >= 0) and (cXa >= 0)
    else:
        return (aXb <= 0) and (bXc <= 0) and (cXa <= 0)

def PointNearTriangle(d, p, t):
    # If inside, the point is considered "near".
    if PointInTriangle(p, t):
        return True

    # Check proximity to the three sides.  There is definitely
    # redundancy and hence inefficiency in these tests.
    return (PointNearSegment(d, p, (t[0], t[1])) or
            PointNearSegment(d, p, (t[1], t[2])) or
            PointNearSegment(d, p, (t[2], t[0])))

#
#   Intersection of Segment and Triangle
#
def SegmentCrossTriangle(s, t):
    return (PointInTriangle(s[0], t) or
            PointInTriangle(s[1], t) or
            SegmentCrossSegment(s, (t[0], t[1])) or
            SegmentCrossSegment(s, (t[1], t[2])) or
            SegmentCrossSegment(s, (t[2], t[0])))


######################################################################
#
#   RELATIVE TO BOX
#
#   Check validity (convexity of box)
#
def ValidBox(b):
    return SegmentCrossSegment((b[0], b[2]), (b[1], b[3]))

#
#   Proximity of Point to Box
#
def PointInBox(p, b):
    # Precompute the relative vectors.
    (apx, apy) = (b[0][0]-p[0], b[0][1]-p[1])
    (bpx, bpy) = (b[1][0]-p[0], b[1][1]-p[1])
    (cpx, cpy) = (b[2][0]-p[0], b[2][1]-p[1])
    (dpx, dpy) = (b[3][0]-p[0], b[3][1]-p[1])

    # Precompute the cross products
    aXb = apx * bpy - apy * bpx
    bXc = bpx * cpy - bpy * cpx
    cXd = cpx * dpy - cpy * dpx
    dXa = dpx * apy - dpy * apx

    # Determine whether the box is given CW (axb + bxc + cxd + dxa > 0) or CCW.
    if (aXb + bXc + cXd + dXa > 0):
        return (aXb >= 0) and (bXc >= 0) and (cXd >= 0) and (dXa >= 0)
    else:
        return (aXb <= 0) and (bXc <= 0) and (cXd <= 0) and (dXa <= 0)

def PointNearBox(d, p, b):
    # If inside, the point is considered "near".
    if PointInBox(p, b):
        return True

    # Check proximity to the four sides.  There is definitely
    # redundancy and hence inefficiency in these tests.
    return (PointNearSegment(d, p, (b[0], b[1])) or
            PointNearSegment(d, p, (b[1], b[2])) or
            PointNearSegment(d, p, (b[2], b[3])) or
            PointNearSegment(d, p, (b[3], b[0])))

#
#   Intersection of Segment and Box
#
def SegmentCrossBox(s, b):
    return (PointInBox(s[0], b) or
            PointInBox(s[1], b) or
            SegmentCrossSegment(s, (b[0], b[1])) or
            SegmentCrossSegment(s, (b[1], b[2])) or
            SegmentCrossSegment(s, (b[2], b[3])) or
            SegmentCrossSegment(s, (b[3], b[0])))


######################################################################
#
#   RELATIVE TO ARC
#
#   Intersection of Segment and Arc
#
def SegmentCrossArc(s, a):
    # Precompute the relative vectors.
    (xmr, ymr) = (0.5*(s[1][0]+s[0][0])-a[0][0], 0.5*(s[1][1]+s[0][1])-a[0][1])
    (dx , dy ) = (0.5*(s[1][0]-s[0][0])        , 0.5*(s[1][1]-s[0][1])        )
    (xrb, yrb) = (a[0][0]-a[2][0], a[0][1]-a[2][1])
    (xab, yab) = (a[1][0]-a[2][0], a[1][1]-a[2][1])

    # Precompute the constants
    a = dx**2  + dy**2
    b = dx*xmr + dy*ymr

    # Check where the are any possible line/circle crossings.
    square = - (dx*ymr - dy*xmr)**2 + a*(xrb**2+yrb**2)
    if not (square >= 0.0):
        return False
    root = np.sqrt(square)

    # Determine the two possible crossings.
    n1 = (-b+root)/a
    n2 = (-b-root)/a

    # Check the two possible crossings fall in valid regions.
    g = xab*(ymr+yrb) - yab*(xmr+xrb)
    h = xab*dy        - yab*dx
    return (((abs(n1) <= 1) and (g+n1*h >= 0)) or
            ((abs(n2) <= 1) and (g+n2*h >= 0)))
