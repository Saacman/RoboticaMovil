import numpy as np
from PIL import Image
from PIL.PngImagePlugin import PngInfo
def rotx(theta):
    Rx = np.array([[1,0,0],[0,np.cos(theta),-np.sin(theta)],[0,np.sin(theta),np.cos(theta)]])
    return Rx

def roty(theta):
    Ry = np.array([[np.cos(theta),0,np.sin(theta)],[0,1,0],[-np.sin(theta),0,np.cos(theta)]])
    return Ry

def rotz(theta):
    Rz = np.array([[np.cos(theta),-np.sin(theta),0],[np.sin(theta),np.cos(theta),0],[0,0,1]])
    return Rz

def transformB2A(euler, trans, pos_b):
    """
    Transform point from system B to A given
    the translation and rotation angles
    """
    alpha = euler[0]
    beta = euler[1]
    gamma = euler[2]
    AB_R = np.dot(rotx(alpha), np.dot(roty(beta), rotz(gamma)))
    rotated = np.dot(AB_R, pos_b.reshape(3,1))
    return (rotated + np.array(trans).reshape(3,1)).reshape(3)


class GridMap:
    def __init__(self, init_pos, center=np.array([0,0]), arr = np.ones((10,10)) * 0.5, tile_size = 0.1):
        self.coffset = center
        self.grid = arr
        self.tsize = tile_size
        self.ipos = init_pos
        
    @classmethod
    def loadImg(cls, path, tsize = 0.1):
        """
        Read information from an image
        """
        img = Image.open(path)

        if img.info:
            tsize=float(img.info["tsize"])
            ipos = np.array([float(img.info["iposx"]), float(img.info["iposy"])])
        else:
            ipos = np.zeros(2)

        arr = np.array(img).astype(np.float) / 255
        arr = np.flipud(np.around(arr, 1))
        if 0.8 in arr:
            r = np.where(arr == 0.8)
            center = np.array([r[1][0], r[0][0]])
            arr[center[1], center[0]] = 0
        else:
            center = np.array((0,0))
        obj = cls(ipos, center, arr, tsize)
        return obj

    def saveImg(self, path):
        """
        Save information as an image
        """
        arr = self.grid
        arr[self.coffset[1], self.coffset[0]] = 0.8
        # Save map info
        meta = PngInfo()
        meta.add_text("iposx", str(self.ipos[0]))
        meta.add_text("iposy", str(self.ipos[1]))
        meta.add_text("tsize", str(self.tsize))
        arr = np.flipud(arr)        
        img = Image.fromarray(np.uint8(arr * 255), 'L')
        img.save(path, "PNG", pnginfo=meta)

        
    def setPoint(self, point, value):
        """
        Save the given value at the equivalent position in the array
        """
        point = point[:2]
        p = point + self.coffset
        # Add columns to the left, if needed
        while p[0] < 0:
            #n = abs(p[0])
            # Add 10 columns at once, to avoid using this block too often
            bffr = np.ones((self.grid.shape[0], 10)) * 0.5
            self.grid = np.concatenate((bffr, self.grid), axis=1)
            # Update the offset
            self.coffset[0] = self.coffset[0] + 10
            # Try again
            p = point + self.coffset
        # Add rows under the current array, if needed
        while p[1] < 0:
            bffr = np.ones((10, self.grid.shape[1])) * 0.5
            self.grid = np.concatenate((bffr, self.grid), axis=0)
            # Update the offset
            self.coffset[1] = self.coffset[1] + 10
            # Try again
            p = point + self.coffset
        # Add columns to the Right, if needed
        while p[0] >= self.grid.shape[1]:
            bffr = np.ones((self.grid.shape[0], 10)) * 0.5
            self.grid = np.concatenate((self.grid, bffr), axis=1)
            
        # Add rows above the current array, if needed
        while p[1] >= self.grid.shape[0]:
            bffr = np.ones((10, self.grid.shape[1])) * 0.5
            self.grid = np.concatenate((self.grid, bffr), axis=0)

        # Finally, add the value to the grid
        self.grid[p[1], p[0]] = value
            
    def getGrid(self):
        """
        Give a view of the saved array ready for display
        """
        return np.flipud(self.grid)
