import numpy as np
import matplotlib.pyplot as plt

def MSE(x,ref):
    err = x-ref
    y = np.square(err)
    return np.abs(err), y

def MSLE(x,ref):
    err = x-ref
    lerr = np.log(1+x)-np.log(1+ref)
    y = np.square(lerr)
    return np.abs(err), y

def CrossEntropy(x,ref):
    err = x-ref
    a = np.random
    y = (err**2)/np.exp(a.rand(np.size(x))) - (a.rand(np.size(x)))
    return np.abs(err), y

if __name__ == "__main__":
    x = np.array(range(11))
    ref = np.zeros(11)

    err,mse = MSE(x,ref)
    err,msle= MSLE(x,ref)
    err,ce= CrossEntropy(x,ref)

    fontsize = 10
    plt.figure()
    plt.plot(err,err,color='blue',label="Absolute Error")
    plt.plot(err,mse,color='#ff8c00',label="Squared Error")
    # plt.plot(err,msle,color='g',label="Squared Logarithmic Error")
    plt.plot(err,ce,color='k',label="Cross Entropy")
    plt.xticks(fontsize=fontsize)
    plt.yticks(fontsize=fontsize)
    plt.xlabel('Absolute Error', size=fontsize)
    plt.ylabel("Value", size=fontsize)
    plt.grid()
    plt.legend()
    plt.show()