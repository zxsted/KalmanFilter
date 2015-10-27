import numpy as np


class KalmanFilter:
    """Simple Multivariate Kalman Filter-
       
       Make sure all values are input as 2D arrays, even if just a scalar.
       
    """


    def __init__(self, x, F, P, Q=1, R=1, H=1):
        
        
        
        self.__x = x    #State Matrix
        self.__F = F    #State Transistion Matrix
        self.__P = P    #Initial Uncertainty Covariance Matrix
        self.__Q = Q    #Process Model Uncertainty Covariance Matrix
        self.__R = R    #Measurement Covariance Matrix
        self.__H = H    #Measurement Transition Matrix
    
        #These don't really need to be data members, but they are set for debug purposes
    
        #self.__K  =1  #Kalman Gain
        #self.__y  =1  #Residual
        #self.__S  =1  #System uncertainty projected in measurement space(whatever that means)
    
    
    def predict(self, dt):
        
        self.__x = np.dot(self.__F, self.__x) #Control inputs will be added later
        
        self.__P = np.dot(np.dot(self.__F, self.__P), self.__F.T) #+ self.__Q

    def update(self, z):

        #Residual Computation
        y = z - np.dot(self.__H, self.__x)
        
        S = np.dot(self.__H, self.__P).dot(self.__H.T) + self.__R
        
        K = np.dot(self.__P, self.__H.T).dot(np.linalg.inv(S))
        
        self.__x = self.__x + np.dot(K, y)
        
        self.__P = (np.eye(2)-np.dot(K, self.__H)).dot(self.__P)
        

    def get_state(self):
        return self.__x
    
    def get_P(self):
        return self.__P
        
        
 
if __name__ == "__main__":
    
    import numpy as np
    dt = 1

    x = np.array([[0],[1]])
    F = np.array([[1,dt],[0,1]])
    P = np.diag([5,5])
    H = np.array([[1,0]])
    R = np.array([[2]])

    KF = KalmanFilter(x,F,P,1,R,H)
    for i in range(10):
        KF.predict(dt)
        KF.update(np.array([[i+1]]))
        print("New State:\n",KF.get_state(),"\n")
        print("System Var:\n", KF.get_P(),"\n")
    
    
    
    
    