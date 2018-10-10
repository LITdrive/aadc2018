# LITD-ALARM-CLF:
#
# run from /AADC/src/aadcUserPython/litdrive$
# anaconda base envrionment
# python -m litdrive.filters.alarm

import time
import numpy as np
import sounddevice as sd
import matplotlib.pyplot as plt

from sklearn.externals import joblib
from keras.models import load_model
from os.path import dirname, join, abspath

from ...zeromq.server import ZmqServer

#Parameters--------------
p =  abspath(join(dirname(__file__), r'../../../../../../configuration_files/alarm'))
thld = 0.65 #treshold for detection
weights = np.asarray([7,10,9,8]) #weights for final dec. lr, es, cnn, nb_pca
weights=weights/sum(weights)
is_debugging=False #debugging mode
# don't touch those ;)
fs = 48*10**3 #48k
nfft = 2**11
buff_len = 1
duration = 1.48 #has to be longer than 1.48 seconds
f_min = 1000
f_max = 3000

scaling_mean, scaling_std = 7, 1 #old was 15, 3 (with matplotlib specgram)

#------------------------

sd.default.samplerate = fs
sd.default.channels = 1

# load the clf
clf = joblib.load(join(p, 'alarm_lr.pkl'))#'alarm_lr_clf2.pkl')
cnn = load_model(join(p, 'alarm_cnn.h5'))#"cnn_new.h5")
nb = joblib.load(join(p, 'alarm_nb_pca.pkl'))#'nb_pca.pkl')
pca = joblib.load(join(p, 'alarm_pca.pkl'))

def computeEngineeringSol(spec):
    """
        Compute the engineering solution given the spectrogram for 1.5 seconds
        The frequency-Indizes are pre-definded --> sampling rate of 44k; nfft of 2**11
    """
    a=np.nan_to_num(spec[[73, 96, 97, 98, 121, 122]])
    a-=a.mean()
    a/=a.std() #TODO check for 0 std

    b=np.nan_to_num( np.roll(spec[[71, 88, 89, 90, 91, 123, 124]], 17,axis=1) )
    b-=b.mean()
    b/=b.std()

    c=(a.sum(axis=0)+b.sum(axis=0))
    c-=c.mean()
    c/=c.std()

    d = np.abs( np.convolve(c,[*(np.ones(8)-2),*np.ones(17),*(np.ones(8)-2)]) )
    d = (d)+np.roll(d,26) #make oscilation to per
    return abs( (d.sum()-300)/1000 )

def mySpec(sig,fs,nfft):
    """ 
        Computes the spectrogram given the signal, the frequency of the signal (fs) 
        as well as the nfft (over how many samples the indiviual ffts are done)
        It is windowed as a rechteck ;) due to performance and simplicity ;)
        Returns specgram, freqency-scale, time-scale for the specgram-matrix
    """
    l = range(0,len(sig)-len(sig)%nfft,nfft)
    a = np.zeros((nfft//2,len(l)))
    #sig = sig-sig.mean()
    for si, s in enumerate(l):
        ss = sig[s:s+nfft].flatten()
        a[:,si] = (np.abs(np.fft.fft(ss))/nfft)[:nfft//2]**2 #flatten!!! otherwise it will be done over rows =SSSS
        # fuck that cost me some time =S
    freqs = np.linspace(0,fs/2,(nfft/2))
    t = np.linspace(0,(len(sig)-len(sig)%nfft)/fs,len(l))
    return a, freqs, t
    #plt.imshow((np.log(a)))

def getSireneProb():
    #record
    sig = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    time.sleep(duration) #TODO implement as stream
    
    #compute spectrogram
    #spectrum, freqs, t, im = specgram(sig.flatten(),Fs=fs,NFFT=nfft,scale='dB',noverlap=0)
    spectrum, freqs, t = mySpec(sig.flatten(),fs,nfft)

    freqidx = (freqs>f_min)*(freqs<f_max) #TODO move out of here to make more efficient
    if sum(spectrum[freqidx]<=0)>0: 
        print('No Microphone Signal =S --> get the mic running ;); Set timer to min 1.5 seconds')
        return 0.0
    X = (np.log(spectrum[freqidx])+scaling_mean)/scaling_std #scale
    
    X=np.nan_to_num(X[:85,:34]) #make sure there are no nan's in there
    Xc = X.reshape(1,-1)[:,:2890]
    # linear regression:
    prob = clf.predict_proba((Xc+8)/3)[:,1][0] #makes it a little bit more robust ;)
    # engineering solution:
    prob2 = computeEngineeringSol(spectrum) 
    # cnn takes around 1.5% of cpu
    prob3 = cnn.predict(X.reshape(-1,X.shape[0],X.shape[1],1))[0][0]
    # nb_pca takes um 3-4% of cpu =S but is pretty good
    prob4 = nb.predict_proba(pca.transform(Xc))[:,1][0]
    prbs = np.asarray([prob,prob2,prob3,prob4])
    
    fin = np.dot( prbs, weights.T )

    txt = 'lr:{:3.0f}% es:{:3.0f}% cnn:{:3.0f}% nb:{:3.0f}% finDecision:{:3.0f}% {}'.format(
          prob*100, prob2*100,prob3*100,prob4*100,fin*100,fin>0.6) #/r
    print(txt, end='\n')

    if is_debugging:
        if fin>0.5:
            plt.imshow(X)
            plt.title(txt)
            plt.show()

    return fin

def process(signal):
    #print('running')
    thld = 0.6
    bool_out = (1, getSireneProb()>thld)
    return [bool_out] #return list, or mutliple elements.. otherwise it will crash


if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5563",
                    ["tSignalValue",], 
                    ["tBoolSignalValue",])
    try:
        zmq.connect()
        zmq.run(process)
    finally:
        zmq.disconnect()
