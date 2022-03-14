# second attempt at planar utilties functions
#
# Focus on segments

from utilities import *
import numpy as np

'''
Returns point(s) where s1 crosses s2. Three cases:

1. Segments intersect at 1 point -> returns (pt1, -1)
2. Segments are parallel -> returns (pt1, pt2) describing line of intersection
3. Segments do not intersect -> returns -1

Ref: https://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
'''
def WhereSegmentCrossSegment(s1, s2):
    ## precompute vectors
    p = s1.pt1
    r = s1.pt2 - s1.pt1

    q = s2.pt1
    s = s2.pt2 - s2.pt1

    ## Compute intersection cases for u
    # (q - p) x r
    num = (q - p).cross(r)
    # (r x s)
    denom = r.cross(s)

    ## Cases
    # COLINEAR
    if (num == 0 and denom == 0):
        t0 = (q - p).dot(r) / (r.dot(r))
        t1 = t0 + s.dot(r) / (r.dot(r))

        # lines intersect somewhere if [t0, t1] contains [0, 1]
        if (t0 < 0 and t1 < 0):
            # no intersection
            return -1
        if (t0 > 1 and t1 > 1):
            # no intersection
            return -1

        # return endpoints of intersection line
        if (t0 < 0):
            # must intersect from t = 0 to min(1, t1)
            int1 = p # + 0*r
            int2 = p + r.scale(min(1, t1))
            return (int1, int2)
        if (t0 > 1):
            # must intersect from max(0, t1) to t = 1
            int1 = p + r.scale(max(0, t1))
            int2 = p + r
            return (int1, int2)
        if (t1 < 0):
            # must intersect from t = 0 to min(1, t0)
            int1 = p # + 0*r
            int2 = p + r.scale(min(1,t0))
            return (int1, int2)
        if (t1 > 1):
            # must intersect from max(0, t0) to t = 1
            int1 = p + r.scale(max(0, t0))
            int2 = p + r
            return (int1, int2)

        # [t0, t1] is inside [0, 1]
        int1 = p + r.scale(t0)
        int2 = p + r.scale(t1)
        return (int1, int2)

    # PARALLEL BUT NOT COLINEAR
    if (denom == 0 and num != 0):
        # no intersection
        return -1

    # INTERSECT AT ONE POINT
    if (denom != 0):
        # calculate u and t
        u = num / denom
        t = (q - p).cross(s) / denom

        # check that 0 <= t <= 1  and 0 <= u <= 1
        if (u >= 0 and u <= 1):
            if (t >= 0 and t <= 1):
                int1 = p + r.scale(u)
                int2 = -1
                return (int1, int2)

        # if u and t are not correctly bounded, lines are not parallel and not intersecting
        return -1

'''
Returns segments in rectangle computed from a centerline.
Computes segments with endpoints positively oriented.

A       D
---------
B       C

Returns (AB, BC, CD, DA)

'''
def RectangleFromCenterline(cl, width):
    # compute slope of centerline
    dx = cl.xdif()
    dy = cl.ydif()

    # check cases when slope is singular
    if (dx == 0):
        # the line is vertical
        A = Point(cl.pt1.x - width, cl.pt1.y)
        B = Point(cl.pt1.x + width, cl.pt1.y)
        C = Point(cl.pt2.x + width, cl.pt2.y)
        D = Point(cl.pt2.x - width, cl.pt2.y)

        return (Segment(A, B), Segment(B, C), Segment(C, D), Segment(D, A))

    if (dy == 0):
        # the line is horizontal
        A = Point(cl.pt1.x, cl.pt1.y + width)
        B = Point(cl.pt1.x, cl.pt1.y - width)
        C = Point(cl.pt2.x, cl.pt2.y - width)
        D = Point(cl.pt2.x, cl.pt2.y + width)

        return (Segment(A, B), Segment(B, C), Segment(C, D), Segment(D, A))

    # line is not vertical/horizontal. Safe to compute slope
    perpslope = - dx/dy
    # get offsets relative to pt1, pt2
    xoffset = np.sqrt(width*width / (1 + perpslope*perpslope))
    yoffset = perpslope*xoffset

    # get endpoints
    A = Point(pt1.x - xoffset, pt1.y - yoffset)
    B = Point(pt1.x + xoffset, pt1.y + yoffset)
    C = Point(pt2.x + xoffset, pt2.y + yoffset)
    D = Point(pt2.x - xoffset, pt2.y - yoffset)

    return (Segment(A, B), Segment(B, C), Segment(C, D), Segment(D, A))


'''
Returns segment where segment intersects rectangle defined by centerline and width.
Uses max distance sensor length to scale down segments

Cases:
1. Segment crosses 2 rectangle edges -> return segment of two intersection points
2. Segment crosses 1 rectangle edge -> return intersection point and endpoint of segment within rectangle
    -> may return same point if segment endpoint is on rectangle edge
3. Segment inside rectange -> return segment
4. Segment outside rectangle -> return -1

Segment reference:
A       D
---------
B       C
'''
def SegmentCrossRectangle(s, centerline, width):
    (AB, BC, CD, DA) = SegmentsFromCenterline(centerline, width)
    rectEdges = (AB, BC, CD, DA)

    # Check if segment crosses any rectangle edge
    iAB = WhereSegmentCrossSegment(s, AB)
    iBC = WhereSegmentCrossSegment(s, BC)
    iCD = WhereSegmentCrossSegment(s, CD)
    iDA = WhereSegmentCrossSegment(s, DA)
    intersections = (iAB, iBC, iCD, iDA)

    # A line cannot intersect at more than two points
    pt1 = -1
    pt2 = -1
    for i in intersections:
        if (i == -1):
            # segment does not intersect this rectangle edge. Move along.
            continue

        if (i[1] == -1):
            # Segment intersects the rectangle edge in 1 place
            # update pt1 or pt2 if either is not up to date
            if (pt1 == -1):
                pt1 = i[0]
            else:
                pt2 = i[0]
            continue

        # i contains 2 points -> segment is parallel to this edge (CASE 1)
        # this is not a huge deal, just return the intersection endpoints
        return Segment(i[0], i[1])

    # all rectangle edges have been searched. If pt1 and pt2 are filled, we are done.
    if (pt2 != -1):
        return Segment(pt1, pt2)

    ## One or more endpoints may be inside the rectangle.
    ## Endpoint is inside if cross product with all rectangle edges is the same sign
    ## Check both endpoints.
    sign1 = 0
    sign2 = 0
    for edge in rectEdges:
        # create vector relative to first point of edge
        edgeref = edge.pt2 - edge.pt1
        pt1ref = s.pt1 - edge.pt1
        pt2ref = s.pt2 - edge.pt2

        sign1 += (edgeref.cross(pt1ref) > 0)
        sign2 += (edgeref.cross(pt1ref) > 0)

    # check if cross products were same sign
    if (pt1 == -1):
        if (abs(sign1) == 4 and abs(sign2) == 4):
            # segment is fully enclosed in rectangle
            return s
        else:
            # segment outside of rectangle
            return -1
    else:
        if (abs(sign1) == 4):
            pt2 = s.pt1
            return Segment(pt1, pt2)
        if (abs(sign2) == 4):
            pt2 = s.pt2
            return Segment(pt1, pt2)

        # segment endpoints are both outside of rectangle, yet segment intersects rectangle.
        # Return segment corresponding to single point
        return Segment(pt1, pt1)




## CHECKING FUNCTIONS
def CheckWhereSegmentCrossSegment():
    # check various WhereSegmentCrossSegment cases
    # PARALLEL
    s1 = Segment(Point(-1,-1), Point(1, 1))
    s2 = Segment(Point(0,0), Point(2, 2))
    print("Nominal: (0, 0), (1, 1)")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")
    # PARALLEL
    s1 = Segment(Point(0,0), Point(1, 1))
    s2 = Segment(Point(0,0), Point(1, 1))
    print("Nominal: (0, 0), (1, 1)")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")
    # PARALLEL
    s1 = Segment(Point(0,0), Point(1, 1))
    s2 = Segment(Point(1,1), Point(2, 2))
    print("Nominal: (1, 1)")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")
    # PARALLEL NO INTERSECT
    s1 = Segment(Point(0,0), Point(0, 1))
    s2 = Segment(Point(1,1), Point(1, 2))
    print("Nominal: -1")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")

    # ONE POINT INTERSECT
    s1 = Segment(Point(0,0), Point(1, 1))
    s2 = Segment(Point(0,1), Point(1, 0))
    print("Nominal: (1/2, 1/2), -1")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")
    # ONE POINT INTERSECT
    s1 = Segment(Point(0,0), Point(1, 10))
    s2 = Segment(Point(0,1), Point(1, 0))
    print("Nominal: NUMBER, -1")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")
    # ONE POINT NO INTERSECT
    s1 = Segment(Point(0,0), Point(1, 10))
    s2 = Segment(Point(1,1), Point(1, 2))
    print("Nominal: -1")
    print("Output:", WhereSegmentCrossSegment(s1, s2))
    print("------")

def main():
    CheckWhereSegmentCrossSegment()



if __name__== "__main__":
    main()
