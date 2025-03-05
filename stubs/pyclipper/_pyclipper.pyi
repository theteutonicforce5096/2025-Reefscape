"""

Cython wrapper for the C++ translation of the Angus Johnson's Clipper
library (ver. 6.2.1) (http://www.angusj.com/delphi/clipper.php)

This wrapper was written by Maxime Chalton, Lukas Treyer and Gregor Ratajc.

"""
from __future__ import annotations
from _frozen_importlib import PyIntRect
import builtins as __builtins__
from collections import namedtuple
import copy as _copy
import numbers as _numbers
import struct as struct
import sys as _sys
import time as _time
import unicodedata as _unicodedata
import warnings as _warnings
__all__ = ['Area', 'CT_DIFFERENCE', 'CT_INTERSECTION', 'CT_UNION', 'CT_XOR', 'CleanPolygon', 'CleanPolygons', 'ClipperException', 'ClosedPathsFromPolyTree', 'ET_CLOSEDLINE', 'ET_CLOSEDPOLYGON', 'ET_OPENBUTT', 'ET_OPENROUND', 'ET_OPENSQUARE', 'JT_MITER', 'JT_ROUND', 'JT_SQUARE', 'MinkowskiDiff', 'MinkowskiSum', 'MinkowskiSum2', 'OpenPathsFromPolyTree', 'Orientation', 'PFT_EVENODD', 'PFT_NEGATIVE', 'PFT_NONZERO', 'PFT_POSITIVE', 'PT_CLIP', 'PT_SUBJECT', 'PointInPolygon', 'PolyTreeToPaths', 'PyIntRect', 'PyPolyNode', 'Pyclipper', 'PyclipperOffset', 'ReversePath', 'ReversePaths', 'SILENT', 'SimplifyPolygon', 'SimplifyPolygons', 'log_action', 'namedtuple', 'scale_from_clipper', 'scale_to_clipper', 'struct']
class ClipperException(Exception):
    pass
class PyPolyNode:
    """
    
        Represents ClipperLibs' PolyTree and PolyNode data structures.
        
    """
    def __init__(self):
        ...
class Pyclipper:
    """
    Wraps the Clipper class.
    
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/Clipper/_Body.htm
        
    """
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    def AddPath(self, path, poly_type, closed = True):
        """
         Add individual path.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperBase/Methods/AddPath.htm
        
                Keyword arguments:
                path      -- path to be added
                poly_type -- type of the added path - subject or clip
                closed    -- True if the added path is closed, False if open
        
                Returns:
                True -- path is valid for clipping and was added
        
                Raises:
                ClipperException -- if path is invalid for clipping
                
        """
    def AddPaths(self, paths, poly_type, closed = True):
        """
         Add a list of paths.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperBase/Methods/AddPaths.htm
        
                Keyword arguments:
                paths     -- paths to be added
                poly_type -- type of added paths - subject or clip
                closed    -- True if added paths are closed, False if open
        
                Returns:
                True -- all or some paths are valid for clipping and were added
        
                Raises:
                ClipperException -- all paths are invalid for clipping
                
        """
    def Clear(self):
        """
         Removes all subject and clip polygons.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperBase/Methods/Clear.htm
                
        """
    def Execute(self, clip_type, subj_fill_type = 0, clip_fill_type = 0):
        """
         Performs the clipping operation and returns a list of paths.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/Clipper/Methods/Execute.htm
        
                Keyword arguments:
                clip_type      -- type of the clipping operation
                subj_fill_type -- fill rule of subject paths
                clip_fill_type -- fill rule of clip paths
        
                Returns:
                list of resulting paths
        
                Raises:
                ClipperException -- operation did not succeed
                
        """
    def Execute2(self, clip_type, subj_fill_type = 0, clip_fill_type = 0):
        """
         Performs the clipping operation and returns a PyPolyNode.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/Clipper/Methods/Execute.htm
        
                Keyword arguments:
                clip_type      -- type of the clipping operation
                subj_fill_type -- fill rule of subject paths
                clip_fill_type -- fill rule of clip paths
        
                Returns:
                PyPolyNode
        
                Raises:
                ClipperException -- operation did not succeed
                
        """
    def GetBounds(self):
        """
         Returns an axis-aligned bounding rectangle that bounds all added polygons.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperBase/Methods/GetBounds.htm
        
                Returns:
                PyIntRect with left, right, bottom, top vertices that define the axis-aligned bounding rectangle.
                
        """
class PyclipperOffset:
    """
     Wraps the ClipperOffset class.
    
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/_Body.htm
        
    """
    @staticmethod
    def __new__(type, *args, **kwargs):
        """
        Create and return a new object.  See help(type) for accurate signature.
        """
    @staticmethod
    def __reduce__(*args, **kwargs):
        ...
    @staticmethod
    def __setstate__(*args, **kwargs):
        ...
    def AddPath(self, path, join_type, end_type):
        """
         Add individual path.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/Methods/AddPath.htm
        
                Keyword arguments:
                path      -- path to be added
                join_type -- join type of added path
                end_type  -- end type of added path
                
        """
    def AddPaths(self, paths, join_type, end_type):
        """
         Add a list of paths.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/Methods/AddPaths.htm
        
                Keyword arguments:
                path      -- paths to be added
                join_type -- join type of added paths
                end_type  -- end type of added paths
                
        """
    def Clear(self):
        """
         Clears all paths.
        
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/Methods/Clear.htm
                
        """
    def Execute(self, delta):
        """
         Performs the offset operation and returns a list of offset paths.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/Methods/Execute.htm
        
                Keyword arguments:
                delta -- amount to which the supplied paths will be offset - negative delta shrinks polygons,
                         positive delta expands them.
        
                Returns:
                list of offset paths
                
        """
    def Execute2(self, delta):
        """
         Performs the offset operation and returns a PyPolyNode with offset paths.
                More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Classes/ClipperOffset/Methods/Execute.htm
        
                Keyword arguments:
                delta -- amount to which the supplied paths will be offset - negative delta shrinks polygons,
                         positive delta expands them.
        
                Returns:
                PyPolyNode
                
        """
def Area(poly):
    """
     Get area of the supplied polygon.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Area.htm
    
        Keyword arguments:
        poly -- closed polygon
    
        Returns:
        Positive number if orientation is True
        Negative number if orientation is False
        
    """
def CleanPolygon(poly, distance = 1.415):
    """
     Removes unnecessary vertices from the provided polygon.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/CleanPolygon.htm
    
        Keyword arguments:
        poly     -- polygon to be cleaned
        distance -- distance on which vertices are removed, see 'More info' (default: approx. sqrt of 2)
    
        Returns:
        cleaned polygon
        
    """
def CleanPolygons(polys, distance = 1.415):
    """
     Removes unnecessary vertices from the provided polygons.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/CleanPolygons.htm
    
        Keyword arguments:
        polys    -- polygons to be cleaned
        distance -- distance on which vertices are removed, see 'More info' (default: approx. sqrt of 2)
    
        Returns:
        list of cleaned polygons
        
    """
def ClosedPathsFromPolyTree(poly_node):
    """
     Filters out open paths from the PyPolyNode and returns only closed paths.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/ClosedPathsFromPolyTree.htm
    
        Keyword arguments:
        py_poly_node -- PyPolyNode to be filtered
    
        Returns:
        list of closed paths
        
    """
def MinkowskiDiff(poly1, poly2):
    """
     Performs Minkowski Difference.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/MinkowskiDiff.htm
    
        Keyword arguments:
        poly1 -- polygon
        poly2 -- polygon
    
        Returns:
        list of polygons
        
    """
def MinkowskiSum(pattern, path, path_is_closed):
    """
     Performs Minkowski Addition of the pattern and path.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/MinkowskiSum.htm
    
        Keyword arguments:
        pattern        -- polygon whose points are added to the path
        path           -- open or closed path
        path_is_closed -- set to True if passed path is closed, False if open
    
        Returns:
        list of polygons (containing one or more polygons)
        
    """
def MinkowskiSum2(pattern, paths, path_is_closed):
    """
     Performs Minkowski Addition of the pattern and paths.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/MinkowskiSum.htm
    
        Keyword arguments:
        pattern        -- polygon whose points are added to the paths
        paths          -- open or closed paths
        path_is_closed -- set to True if passed paths are closed, False if open
    
        Returns:
        list of polygons
        
    """
def OpenPathsFromPolyTree(poly_node):
    """
     Filters out closed paths from the PyPolyNode and returns only open paths.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/OpenPathsFromPolyTree.htm
    
        Keyword arguments:
        py_poly_node -- PyPolyNode to be filtered
    
        Returns:
        list of open paths
        
    """
def Orientation(poly):
    """
     Get orientation of the supplied polygon.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/Orientation.htm
    
        Keyword arguments:
        poly -- closed polygon
    
        Returns:
        True  -- counter-clockwise orientation
        False -- clockwise orientation
        
    """
def PointInPolygon(point, poly):
    """
     Determine where does the point lie regarding the provided polygon.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PointInPolygon.htm
    
        Keyword arguments:
        point -- point in question
        poly  -- closed polygon
    
        Returns:
        0  -- point is not in polygon
        -1 -- point is on polygon
        1  -- point is in polygon
        
    """
def PolyTreeToPaths(poly_node):
    """
     Converts a PyPolyNode to a list of paths.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/PolyTreeToPaths.htm
    
        Keyword arguments:
        py_poly_node -- PyPolyNode to be filtered
    
        Returns:
        list of paths
        
    """
def ReversePath(path):
    """
     Reverses the vertex order (and hence orientation) in the specified path.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/ReversePath.htm
    
        Note: Might be more effective to reverse the path outside of this package (eg. via [::-1] on a list)
        so there is no unneeded conversions to internal structures of this package.
    
        Keyword arguments:
        path -- path to be reversed
    
        Returns:
        reversed path
        
    """
def ReversePaths(paths):
    """
     Reverses the vertex order (and hence orientation) in each path.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/ReversePaths.htm
    
        Note: Might be more effective to reverse each path outside of this package (eg. via [::-1] on a list)
        so there is no unneeded conversions to internal structures of this package.
    
        Keyword arguments:
        paths -- paths to be reversed
    
        Returns:
        list if reversed paths
        
    """
def SimplifyPolygon(poly, fill_type = 0):
    """
     Removes self-intersections from the supplied polygon.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/SimplifyPolygon.htm
    
        Keyword arguments:
        poly      -- polygon to be simplified
        fill_type -- PolyFillType used with the boolean union operation
    
        Returns:
        list of simplified polygons (containing one or more polygons)
        
    """
def SimplifyPolygons(polys, fill_type = 0):
    """
     Removes self-intersections from the supplied polygons.
        More info: http://www.angusj.com/delphi/clipper/documentation/Docs/Units/ClipperLib/Functions/SimplifyPolygons.htm
    
        Keyword arguments:
        polys     -- polygons to be simplified
        fill_type -- PolyFillType used with the boolean union operation
    
        Returns:
        list of simplified polygons
        
    """
def __reduce_cython__(self):
    ...
def __setstate_cython__(self, __pyx_state):
    ...
def log_action(description):
    ...
def scale_from_clipper(path_or_paths, scale = 2147483648):
    """
    
        Take a path or list of paths with coordinates represented by ints and scale them back to a fractional
        representation. This function does the inverse of `scale_to_clipper()`.
    
        :param path_or_paths: Either a list of paths or a path. A path is a list of tuples of numbers.
        :param scale: The factor by which to divide coordinates when converting them to floats.
        
    """
def scale_to_clipper(path_or_paths, scale = 2147483648):
    """
    
        Take a path or list of paths with coordinates represented by floats and scale them using the specified factor.
        This function can be user to convert paths to a representation which is more appropriate for Clipper.
    
        Clipper, and thus Pyclipper, uses 64-bit integers to represent coordinates internally. The actual supported
        range (+/- 2 ** 62) is a bit smaller than the maximal values for this type. To operate on paths which use
        fractional coordinates, it is necessary to translate them from and to a representation which does not depend
        on floats. This can be done using this function and it's reverse, `scale_from_clipper()`.
    
        For details, see http://www.angusj.com/delphi/clipper/documentation/Docs/Overview/Rounding.htm.
    
        For example, to perform a clip operation on two polygons, the arguments to `Pyclipper.AddPath()` need to be wrapped
        in `scale_to_clipper()` while the return value needs to be converted back with `scale_from_clipper()`:
    
        >>> pc = Pyclipper()
        >>> path = [[0, 0], [1, 0], [1 / 2, (3 / 4) ** (1 / 2)]] # A triangle.
        >>> clip = [[0, 1 / 3], [1, 1 / 3], [1, 2 / 3], [0, 1 / 3]] # A rectangle.
        >>> pc.AddPath(scale_to_clipper(path), PT_SUBJECT)
        >>> pc.AddPath(scale_to_clipper(clip), PT_CLIP)
        >>> scale_from_clipper(pc.Execute(CT_INTERSECTION))
        [[[0.6772190444171429, 0.5590730146504939], [0.2383135547861457, 0.41277118446305394],
          [0.19245008938014507, 0.3333333330228925], [0.8075499106198549, 0.3333333330228925]]]
    
        :param path_or_paths: Either a list of paths or a path. A path is a list of tuples of numbers.
        :param scale: The factor with which to multiply coordinates before converting rounding them to ints. The default
        will give you a range of +/- 2 ** 31 with a precision of 2 ** -31.
        
    """
CT_DIFFERENCE: int = 2
CT_INTERSECTION: int = 0
CT_UNION: int = 1
CT_XOR: int = 3
ET_CLOSEDLINE: int = 1
ET_CLOSEDPOLYGON: int = 0
ET_OPENBUTT: int = 2
ET_OPENROUND: int = 4
ET_OPENSQUARE: int = 3
JT_MITER: int = 2
JT_ROUND: int = 1
JT_SQUARE: int = 0
PFT_EVENODD: int = 0
PFT_NEGATIVE: int = 3
PFT_NONZERO: int = 1
PFT_POSITIVE: int = 2
PT_CLIP: int = 1
PT_SUBJECT: int = 0
SILENT: bool = True
__test__: dict = {'scale_to_clipper (line 537)': "\n    Take a path or list of paths with coordinates represented by floats and scale them using the specified factor.\n    This function can be user to convert paths to a representation which is more appropriate for Clipper.\n\n    Clipper, and thus Pyclipper, uses 64-bit integers to represent coordinates internally. The actual supported\n    range (+/- 2 ** 62) is a bit smaller than the maximal values for this type. To operate on paths which use\n    fractional coordinates, it is necessary to translate them from and to a representation which does not depend\n    on floats. This can be done using this function and it's reverse, `scale_from_clipper()`.\n\n    For details, see http://www.angusj.com/delphi/clipper/documentation/Docs/Overview/Rounding.htm.\n\n    For example, to perform a clip operation on two polygons, the arguments to `Pyclipper.AddPath()` need to be wrapped\n    in `scale_to_clipper()` while the return value needs to be converted back with `scale_from_clipper()`:\n\n    >>> pc = Pyclipper()\n    >>> path = [[0, 0], [1, 0], [1 / 2, (3 / 4) ** (1 / 2)]] # A triangle.\n    >>> clip = [[0, 1 / 3], [1, 1 / 3], [1, 2 / 3], [0, 1 / 3]] # A rectangle.\n    >>> pc.AddPath(scale_to_clipper(path), PT_SUBJECT)\n    >>> pc.AddPath(scale_to_clipper(clip), PT_CLIP)\n    >>> scale_from_clipper(pc.Execute(CT_INTERSECTION))\n    [[[0.6772190444171429, 0.5590730146504939], [0.2383135547861457, 0.41277118446305394],\n      [0.19245008938014507, 0.3333333330228925], [0.8075499106198549, 0.3333333330228925]]]\n\n    :param path_or_paths: Either a list of paths or a path. A path is a list of tuples of numbers.\n    :param scale: The factor with which to multiply coordinates before converting rounding them to ints. The default\n    will give you a range of +/- 2 ** 31 with a precision of 2 ** -31.\n    "}
