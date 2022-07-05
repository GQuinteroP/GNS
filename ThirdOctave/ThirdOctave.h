#define NFilters 27     //Number of third octave filters
#define NUM_SECTIONS 3  //IIR filter number of sections FilterOrder/2
float calTOB[NFilters]     = {
    -3.5586069f, -1.57931034f, -1.25862069f, 
    +3.02758621f, +3.58275862f, +1.78965517f,
    +2.14482759f, +0.77586207f, +1.07241379f,
    +0.11724138f, +0.99655172f, +0.53448276f,
    -0.00689655f, -1.12413793f, -0.27241379f,
    +0.46551724f, +0.34482759f, -1.90689655f,
    -3.49310345f, -1.12413793f, -1.35517241f,
    -2.25862069f, -3.6f, -4.71724138f,
    -9.45862069f, -8.38965517f, -7.85517241
    };
float gains[NFilters*NUM_SECTIONS]     = {
   +0.0005461746f, +0.0005461746f, +0.0005460256f, 
    +0.0006875445f, +0.0006875445f, +0.0006873083f, 
    +0.0008654902f, +0.0008654902f, +0.0008651161f, 
    +0.0010894655f, +0.0010894655f, +0.0010888729f, 
    +0.0013713624f, +0.0013713624f, +0.0013704236f, 
    +0.0017261364f, +0.0017261364f, +0.0017246498f, 
    +0.0021725914f, +0.0021725914f, +0.0021702377f, 
    +0.0027343610f, +0.0027343610f, +0.0027306355f, 
    +0.0034411377f, +0.0034411377f, +0.0034352425f, 
    +0.0043302053f, +0.0043302053f, +0.0043208809f, 
    +0.0054483488f, +0.0054483488f, +0.0054336074f, 
    +0.0068542254f, +0.0068542254f, +0.0068309354f, 
    +0.0086212987f, +0.0086212987f, +0.0085845329f, 
    +0.0108414534f, +0.0108414534f, +0.0107834740f, 
    +0.0136294151f, +0.0136294151f, +0.0135380998f, 
    +0.0171281248f, +0.0171281248f, +0.0169845372f, 
    +0.0215151869f, +0.0215151869f, +0.0212898552f, 
    +0.0270105228f, +0.0270105228f, +0.0266577974f, 
    +0.0338852555f, +0.0338852555f, +0.0333348326f, 
    +0.0424717888f, +0.0424717888f, +0.0416161940f, 
    +0.0531748049f, +0.0531748049f, +0.0518512242f, 
    +0.0664825961f, +0.0664825961f, +0.0644471943f, 
    +0.0829777345f, +0.0829777345f, +0.0798704922f, 
    +0.1033454910f, +0.1033454910f, +0.0986441523f, 
    +0.1283778101f, +0.1283778101f, +0.1213412955f, 
    +0.1589703560f, +0.1589703560f, +0.1485755593f, 
    +0.1961111426f, +0.1961111426f, +0.1809931993f
};

    
float thirdOctIIR[405]  = {
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9993721247f, -0.9993994236f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9994899035f, -0.9995082021f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9988855124f, -0.9989079237f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9992005825f, -0.9992440343f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9993518591f, -0.9993808866f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9985898733f, -0.9986253977f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9989795685f, -0.9990483522f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9991745949f, -0.9992206693f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9982135296f, -0.9982697964f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9986932278f, -0.9988021255f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9989459515f, -0.9990189672f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9977331161f, -0.9978222251f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9983195066f, -0.9984921813f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9986493587f, -0.9987651110f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9971178770f, -0.9972591400f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9978286028f, -0.9981021285f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9982621670f, -0.9984456301f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9963269234f, -0.9965506792f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9971779585f, -0.9976113439f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9977529049f, -0.9980435371f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9953049421f, -0.9956595302f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9963071346f, -0.9969937801f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9970769882f, -0.9975375533f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9939771891f, -0.9945387244f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9951291084f, -0.9962168932f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9961713552f, -0.9969009757f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9922401905f, -0.9931294918f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9935166836f, -0.9952397943f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9949440956f, -0.9961000085f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9899499416f, -0.9913582206f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9912821054f, -0.9940111041f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9932615757f, -0.9950925708f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9869034290f, -0.9891327620f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9881453514f, -0.9924666286f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9909257889f, -0.9938256145f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9828102589f, -0.9863381386f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9836853743f, -0.9905260205f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9876409769f, -0.9922326207f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9772504568f, -0.9828309417f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9772641659f, -0.9880889654f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9829620123f, -0.9902302623f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9696104527f, -0.9784330726f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9679099321f, -0.9850307703f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9762133360f, -0.9877141714f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9589855671f, -0.9729238153f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9541370869f, -0.9811968207f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9663652182f, -0.9845536351f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9440319538f, -0.9660309553f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9336701632f, -0.9763966203f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9518414736f, -0.9805850983f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9227428436f, -0.9574202895f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9030269384f, -0.9703974128f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.9302262068f, -0.9756036997f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.8921154737f, -0.9466844201f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.8569003344f, -0.9629181027f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.8978205919f, -0.9693523049f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.8476607800f, -0.9333303571f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.7872710228f, -0.9536256194f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.8489854336f, -0.9615070224f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.7827082872f, -0.9167675972f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.6822113991f, -0.9421373606f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.7752057314f, -0.9516561031f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.6874692440f, -0.8962975740f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.5244580507f, -0.9280371070f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.6638412476f, -0.9392677546f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.5479116440f, -0.8711056113f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.2901973724f, -0.9109212756f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.4966905117f, -0.9236351252f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.3447406292f, -0.8402590156f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.9494143724f, -0.8905114532f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.2489420176f, -0.9037714005f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +1.0534024239f, -0.8027116656f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.4711402655f, -0.8669282198f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.8902507424f, -0.8781777024f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.6473995447f, -0.7573174238f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, -0.1595224738f, -0.8414251804f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.3922882974f, -0.8442323208f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, +0.1098185554f, -0.7028488517f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, -0.2475565225f, -0.7960745692f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, -0.9020830393f, -0.8189376593f, 
    +1.0000000000f, +0.0000000000f, -1.0000000000f, -0.5383284688f, -0.6380136013f
};
