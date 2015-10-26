import numpy as np


class KalmanFilter:
    
    def __init__(self, dim_x, F, P, Q=1, R=1, H=1):
        self.__dim_x = dim_x
        
        
        self.__x = np.zeros((dim_x, 1))    #State Matrix
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
    
    def get_state(self):
        return self.__x
    
    def get_P(self):
        return self.__P
        
        
 
if __name__ == "__main__":
    dt = 1.0
    kf = KalmanFilter(2, np.array([[1,dt],[0,1]]), np.array([[5,0],[0,5]]))
    print(kf.get_state())
    print(kf.get_P())
    
    kf.predict(dt)
    
    print(kf.get_state())
    print(kf.get_P())
    