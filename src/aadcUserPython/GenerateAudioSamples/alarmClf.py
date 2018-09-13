import sounddevice as sd
import time
import matplotlib.pyplot as plt
import numpy as np
from pylab import *

#Parameters--------------
fs = 48*10**3 #48k
nfft = 2**11
buff_len = 1
duration = 1.46
f_min = 1000
f_max = 3000

scaling_mean = -15
scaling_std = 3

#------------------------
sd.default.samplerate = fs
sd.default.channels = 1

# load the clf
from sklearn.externals import joblib
clf = joblib.load('alarm_lr_clf2.pkl')

def computeEngineeringSol(spec):
    """
        Compute the engineering solution given the spectrogram for 1.5 seconds
    """
    a= np.log ( spec[[73, 96, 97, 98, 121, 122]] )
    a-=a.mean()
    a/=a.std()

    b=np.log( np.roll(spec[[71, 88, 89, 90, 91, 123, 124]], 17,axis=1) )
    b-=b.mean()
    b/=b.std()

    c=(a.sum(axis=0)+b.sum(axis=0))
    c-=c.mean()
    c/=c.std()

    d = np.abs( np.convolve(c,[*(np.ones(8)-2),*np.ones(17),*(np.ones(8)-2)]) )
    d = (d)+np.roll(d,26) #make oscilation to per
    return (d.sum()-300)/1000

def mySpec(sig,fs,nfft):
    l = range(0,len(sig)-len(sig)%nfft,nfft)
    a = np.zeros((nfft//2,len(l)))
    #sig = sig-sig.mean()
    for si, s in enumerate(l):
        ss = sig[s:s+nfft].flatten()
        a[:,si] = (np.abs(np.fft.fft(ss))/nfft)[:nfft//2] #flatten!!! otherwise it will be done over rows =SSSS
        # fuck that cost me some time =S
    freqs = np.linspace(0,fs/2,(nfft/2))
    t = np.linspace(0,(len(sig)-len(sig)%nfft)/fs,len(l))
    return a, freqs, t
    #plt.imshow((np.log(a)))

while True:
    #record
    sig = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    time.sleep(duration)
    
    #compute spectrogram
    
    #spectrum, freqs, t, im = specgram(sig.flatten(),Fs=fs,NFFT=nfft,scale='dB',noverlap=0)
    spectrum, freqs, t = mySpec(sig.flatten(),fs,nfft)
    
    freqidx = (freqs>f_min)*(freqs<f_max)
    X = (np.log(spectrum[freqidx])+scaling_mean)/scaling_std #scale
    
    #spectrum = mySpec(sig)
    #X = np.log(spectrum[43:128])
    #X-=X.mean()
    #X/=X.std()
    #X = ()#+16.7)/2.25
    
    #plt.imshow(X)
    
    X=np.nan_to_num(X) #make sure there are no nan's in there
    prob = clf.predict_proba(X.reshape(1,-1)[:,:2890])[:,1]
    
    # engineering solution:
    prob2 = computeEngineeringSol(spectrum)
    
    fin = (prob+prob2)
    
    print('lr: {:4.0f}% engineeringSol: {:4.0f} finDecision: {}'.format(prob[0]*100, prob2*100,fin>0.5), end='\n')