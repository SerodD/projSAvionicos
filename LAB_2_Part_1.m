
%% Dados de Entrada
% Pressure [mbar] Output [Volt] at -45ºC Output [Volt] at 25ºC Output [Volt] at 125ºC

x = [1.0000000e+01 9.6891912e-02 3.9214138e-02 8.0487284e-02;
 1.5000000e+01 6.0851900e-02 2.2426346e-02 7.6988551e-02;
 2.0000000e+01 3.7395234e-02 1.0771168e-01 1.5599585e-02;
 2.5000000e+01 1.1393540e-01 4.9464483e-02 5.9137552e-02;
 3.0000000e+01 2.3785157e-02 4.4588951e-02 1.4701188e-01;
 3.5000000e+01 1.1012565e-01 8.0013170e-02 3.7981724e-02;
 4.0000000e+01 7.5780823e-02 1.4446871e-01 1.0942118e-01;
 4.5000000e+01 1.1891421e-01 9.1045177e-02 5.2457714e-02;
 5.0000000e+01 1.5745503e-01 1.2253808e-01 1.2698883e-01;
 5.5000000e+01 1.8132342e-01 7.9968573e-02 1.4443765e-01;
 6.0000000e+01 1.4325421e-01 1.5679809e-01 1.3689275e-01;
 6.5000000e+01 1.5888049e-01 1.4951198e-01 1.9430109e-01;
 7.0000000e+01 8.3664964e-02 1.5989808e-01 1.6326326e-01;
 7.5000000e+01 1.3893389e-01 1.9675427e-01 2.2881573e-01;
 8.0000000e+01 2.9442284e-01 2.0526728e-01 2.3766409e-01;
 8.5000000e+01 2.3971441e-01 2.1739275e-01 1.9510525e-01;
 9.0000000e+01 2.6161016e-01 3.5119408e-01 2.2929842e-01;
 9.5000000e+01 2.0265181e-01 2.8627189e-01 2.4003381e-01;
 1.0000000e+02 2.5130987e-01 3.2559117e-01 2.8326019e-01;
 1.0500000e+02 2.0100899e-01 3.3835876e-01 3.0732601e-01;
 1.1000000e+02 2.3209178e-01 2.7399005e-01 3.0223021e-01;
 1.1500000e+02 2.4903023e-01 2.7989896e-01 2.8453107e-01;
 1.2000000e+02 2.4876790e-01 2.4659760e-01 2.6097740e-01;
 1.2500000e+02 2.8395577e-01 2.8843658e-01 2.5828123e-01;
 1.3000000e+02 3.0457423e-01 3.3019425e-01 3.6741545e-01;
 1.3500000e+02 3.3769783e-01 3.5221405e-01 4.1688049e-01;
 1.4000000e+02 4.3349382e-01 3.8328499e-01 3.6107504e-01;
 1.4500000e+02 4.7950391e-01 3.9089416e-01 3.6105251e-01;
 1.5000000e+02 3.8302689e-01 3.5157274e-01 4.1511855e-01;
 1.5500000e+02 4.4169029e-01 4.0075645e-01 3.2382861e-01;
 1.6000000e+02 4.3875983e-01 4.9826069e-01 3.9475517e-01;
 1.6500000e+02 4.1367293e-01 4.3147364e-01 3.3891002e-01;
 1.7000000e+02 4.3542224e-01 4.2159795e-01 4.5036563e-01;
 1.7500000e+02 4.1684755e-01 4.6336205e-01 4.4848812e-01;
 1.8000000e+02 5.4300436e-01 5.0220163e-01 4.7474419e-01;
 1.8500000e+02 5.2261386e-01 4.8840507e-01 4.5868109e-01;
 1.9000000e+02 5.8287927e-01 5.0214433e-01 4.1540932e-01;
 1.9500000e+02 5.2595976e-01 5.2093027e-01 5.4466308e-01;
 2.0000000e+02 5.5587122e-01 5.9806722e-01 4.9306779e-01;
 2.0500000e+02 6.0978227e-01 5.9969556e-01 5.4118960e-01;
 2.1000000e+02 6.0415619e-01 5.5144011e-01 5.8692065e-01;
 2.1500000e+02 6.7901095e-01 5.6742969e-01 5.8657304e-01;
 2.2000000e+02 6.0326682e-01 5.4332639e-01 5.4529824e-01;
 2.2500000e+02 7.1431390e-01 6.7475320e-01 5.8306869e-01;
 2.3000000e+02 6.5832400e-01 6.4116963e-01 5.9992285e-01;
 2.3500000e+02 6.8276307e-01 6.5195701e-01 6.6358042e-01;
 2.4000000e+02 6.8756434e-01 6.8521723e-01 6.9070651e-01;
 2.4500000e+02 7.0113357e-01 6.4207908e-01 6.1178181e-01;
 2.5000000e+02 8.0729867e-01 6.7381584e-01 6.2906698e-01;
 2.5500000e+02 7.2529302e-01 7.3489232e-01 6.9063295e-01;
 2.6000000e+02 8.2617790e-01 7.3125693e-01 7.0094966e-01;
 2.6500000e+02 7.8676380e-01 6.7819597e-01 7.2479305e-01;
 2.7000000e+02 8.0729231e-01 7.4678981e-01 6.9039919e-01;
 2.7500000e+02 8.3369182e-01 8.0549796e-01 6.9457260e-01;
 2.8000000e+02 8.4272344e-01 8.5502937e-01 7.6117973e-01;
 2.8500000e+02 8.1054515e-01 8.3266021e-01 8.3007223e-01;
 2.9000000e+02 8.4176063e-01 8.5220930e-01 8.2197750e-01;
 2.9500000e+02 9.1730853e-01 8.4441212e-01 8.0802999e-01;
 3.0000000e+02 9.0696714e-01 8.8965211e-01 8.1371675e-01;
 3.0500000e+02 9.4392551e-01 9.2059091e-01 8.0045144e-01;
 3.1000000e+02 9.8210881e-01 8.8007419e-01 8.9814998e-01;
 3.1500000e+02 9.7959622e-01 9.7656196e-01 9.1292885e-01;
 3.2000000e+02 9.7497178e-01 8.8513024e-01 8.8859355e-01;
 3.2500000e+02 1.0078278e+00 9.6812736e-01 9.2757359e-01;
 3.3000000e+02 1.0845392e+00 9.7870156e-01 9.3979664e-01;
 3.3500000e+02 1.0760546e+00 1.0724751e+00 9.8302631e-01;
 3.4000000e+02 1.1003147e+00 1.0117897e+00 1.0287173e+00;
 3.4500000e+02 1.1236023e+00 1.1728402e+00 9.0922263e-01;
 3.5000000e+02 1.1449616e+00 1.0501508e+00 9.8473697e-01;
 3.5500000e+02 1.0953951e+00 1.0508223e+00 1.0111162e+00;
 3.6000000e+02 1.2453221e+00 1.1921460e+00 9.8045213e-01;
 3.6500000e+02 1.1591436e+00 1.1162843e+00 1.0375096e+00;
 3.7000000e+02 1.1866896e+00 1.1255241e+00 1.0993561e+00;
 3.7500000e+02 1.1446719e+00 1.1606293e+00 1.0539101e+00;
 3.8000000e+02 1.2682078e+00 1.1333953e+00 1.0176763e+00;
 3.8500000e+02 1.2677326e+00 1.1340540e+00 1.0789134e+00;
 3.9000000e+02 1.2726746e+00 1.1861120e+00 1.1492776e+00;
 3.9500000e+02 1.3099809e+00 1.2342021e+00 1.1251078e+00;
 4.0000000e+02 1.3402323e+00 1.2968175e+00 1.2118209e+00;
 1.6500000e+02 4.1367293e-01 4.3147364e-01 3.3891002e-01;
 1.7000000e+02 4.3542224e-01 4.2159795e-01 4.5036563e-01;
 1.7500000e+02 4.1684755e-01 4.6336205e-01 4.4848812e-01;
 1.8000000e+02 5.4300436e-01 5.0220163e-01 4.7474419e-01;
 1.8500000e+02 5.2261386e-01 4.8840507e-01 4.5868109e-01;
 1.9000000e+02 5.8287927e-01 5.0214433e-01 4.1540932e-01;
 1.9500000e+02 5.2595976e-01 5.2093027e-01 5.4466308e-01;
 2.0000000e+02 5.5587122e-01 5.9806722e-01 4.9306779e-01;
 2.0500000e+02 6.0978227e-01 5.9969556e-01 5.4118960e-01;
 2.1000000e+02 6.0415619e-01 5.5144011e-01 5.8692065e-01;
 2.1500000e+02 6.7901095e-01 5.6742969e-01 5.8657304e-01;
 2.2000000e+02 6.0326682e-01 5.4332639e-01 5.4529824e-01;
 2.2500000e+02 7.1431390e-01 6.7475320e-01 5.8306869e-01;
 2.3000000e+02 6.5832400e-01 6.4116963e-01 5.9992285e-01;
 2.3500000e+02 6.8276307e-01 6.5195701e-01 6.6358042e-01;
 2.4000000e+02 6.8756434e-01 6.8521723e-01 6.9070651e-01;
 2.4500000e+02 7.0113357e-01 6.4207908e-01 6.1178181e-01;
 2.5000000e+02 8.0729867e-01 6.7381584e-01 6.2906698e-01;
 2.5500000e+02 7.2529302e-01 7.3489232e-01 6.9063295e-01;
 2.6000000e+02 8.2617790e-01 7.3125693e-01 7.0094966e-01;
 2.6500000e+02 7.8676380e-01 6.7819597e-01 7.2479305e-01;
 2.7000000e+02 8.0729231e-01 7.4678981e-01 6.9039919e-01;
 2.7500000e+02 8.3369182e-01 8.0549796e-01 6.9457260e-01;
 2.8000000e+02 8.4272344e-01 8.5502937e-01 7.6117973e-01;
 2.8500000e+02 8.1054515e-01 8.3266021e-01 8.3007223e-01;
 2.9000000e+02 8.4176063e-01 8.5220930e-01 8.2197750e-01;
 2.9500000e+02 9.1730853e-01 8.4441212e-01 8.0802999e-01;
 3.0000000e+02 9.0696714e-01 8.8965211e-01 8.1371675e-01;
 3.0500000e+02 9.4392551e-01 9.2059091e-01 8.0045144e-01;
 3.1000000e+02 9.8210881e-01 8.8007419e-01 8.9814998e-01;
 3.1500000e+02 9.7959622e-01 9.7656196e-01 9.1292885e-01;
 3.2000000e+02 9.7497178e-01 8.8513024e-01 8.8859355e-01;
 3.2500000e+02 1.0078278e+00 9.6812736e-01 9.2757359e-01;
 3.3000000e+02 1.0845392e+00 9.7870156e-01 9.3979664e-01;
 3.3500000e+02 1.0760546e+00 1.0724751e+00 9.8302631e-01;
 3.4000000e+02 1.1003147e+00 1.0117897e+00 1.0287173e+00;
 3.4500000e+02 1.1236023e+00 1.1728402e+00 9.0922263e-01;
 3.5000000e+02 1.1449616e+00 1.0501508e+00 9.8473697e-01;
 3.5500000e+02 1.0953951e+00 1.0508223e+00 1.0111162e+00;
 3.6000000e+02 1.2453221e+00 1.1921460e+00 9.8045213e-01;
 3.6500000e+02 1.1591436e+00 1.1162843e+00 1.0375096e+00;
 3.7000000e+02 1.1866896e+00 1.1255241e+00 1.0993561e+00;
 3.7500000e+02 1.1446719e+00 1.1606293e+00 1.0539101e+00;
 3.8000000e+02 1.2682078e+00 1.1333953e+00 1.0176763e+00;
 3.8500000e+02 1.2677326e+00 1.1340540e+00 1.0789134e+00;
 3.9000000e+02 1.2726746e+00 1.1861120e+00 1.1492776e+00;
 3.9500000e+02 1.3099809e+00 1.2342021e+00 1.1251078e+00;
 4.0000000e+02 1.3402323e+00 1.2968175e+00 1.2118209e+00;
 4.0500000e+02 1.3403999e+00 1.3191289e+00 1.1695201e+00;
 4.1000000e+02 1.2454517e+00 1.3092948e+00 1.2194464e+00;
 4.1500000e+02 1.3629791e+00 1.3578840e+00 1.2045355e+00;
 4.2000000e+02 1.3814131e+00 1.4351252e+00 1.2722551e+00;
 4.2500000e+02 1.4263383e+00 1.3754974e+00 1.1995648e+00;
 4.3000000e+02 1.4178686e+00 1.4080715e+00 1.3498807e+00;
 4.3500000e+02 1.4567605e+00 1.3771236e+00 1.2938667e+00;
 4.4000000e+02 1.4760224e+00 1.4578079e+00 1.3158674e+00;
 4.4500000e+02 1.4864560e+00 1.3842383e+00 1.2830165e+00;
 4.5000000e+02 1.5418370e+00 1.4367199e+00 1.3591204e+00;
 4.5500000e+02 1.5397229e+00 1.4358786e+00 1.3753853e+00;
 4.6000000e+02 1.5930214e+00 1.4685477e+00 1.3489256e+00;
 4.6500000e+02 1.5558330e+00 1.5921036e+00 1.4404559e+00;
 4.7000000e+02 1.6650095e+00 1.5242195e+00 1.5165726e+00;
 4.7500000e+02 1.5892261e+00 1.5361324e+00 1.5029593e+00;
 4.8000000e+02 1.6337969e+00 1.6160139e+00 1.4720947e+00;
 4.8500000e+02 1.6080958e+00 1.5825818e+00 1.5165553e+00;
 4.9000000e+02 1.6637974e+00 1.6294014e+00 1.5223973e+00;
 4.9500000e+02 1.7625971e+00 1.6599902e+00 1.5503976e+00;
 5.0000000e+02 1.7533439e+00 1.6240176e+00 1.4704569e+00;
 5.0500000e+02 1.7187081e+00 1.6661290e+00 1.5368296e+00;
 5.1000000e+02 1.7998150e+00 1.7795552e+00 1.5817848e+00;
 5.1500000e+02 1.7735000e+00 1.7291126e+00 1.6387174e+00;
 5.2000000e+02 1.7407340e+00 1.7819508e+00 1.6480677e+00;
 5.2500000e+02 1.8264318e+00 1.8186568e+00 1.6495629e+00;
 5.3000000e+02 1.9507000e+00 1.8123328e+00 1.5873002e+00;
 5.3500000e+02 1.9028974e+00 1.8160909e+00 1.6371953e+00;
 5.4000000e+02 1.8382566e+00 1.8849519e+00 1.7686536e+00;
 5.4500000e+02 1.9693578e+00 1.8044160e+00 1.7410226e+00;
 5.5000000e+02 2.0427824e+00 1.9264414e+00 1.7568788e+00;
 5.5500000e+02 1.9862435e+00 1.8148743e+00 1.7634043e+00;
 5.6000000e+02 2.0055732e+00 1.8713882e+00 1.9144104e+00;
 5.6500000e+02 2.0595954e+00 1.9277803e+00 1.7912589e+00;
 5.7000000e+02 2.0259404e+00 1.9643012e+00 1.8340687e+00;
 5.7500000e+02 2.0955756e+00 1.9245871e+00 1.8824278e+00;
 5.8000000e+02 2.2094876e+00 2.0448180e+00 1.9352076e+00;
 5.8500000e+02 2.0962727e+00 2.0332965e+00 1.8981255e+00;
 5.9000000e+02 2.1587403e+00 2.0151148e+00 1.9635482e+00;
 5.9500000e+02 2.1968645e+00 2.0344298e+00 1.9861486e+00;
 6.0000000e+02 2.2225279e+00 2.0670123e+00 1.9782699e+00;
 6.0500000e+02 2.2643459e+00 2.0954085e+00 1.9366598e+00;
 6.1000000e+02 2.2456164e+00 2.1475712e+00 2.0433766e+00;
 6.1500000e+02 2.2488184e+00 2.1740539e+00 2.0371033e+00;
 6.2000000e+02 2.3011140e+00 2.1761884e+00 2.1170965e+00;
 6.2500000e+02 2.2511727e+00 2.2202104e+00 2.0431432e+00;
 6.3000000e+02 2.3450096e+00 2.2212788e+00 2.0987486e+00;
 6.3500000e+02 2.4682623e+00 2.2785595e+00 2.1008460e+00;
 6.4000000e+02 2.3817363e+00 2.3580751e+00 2.0694914e+00;
 6.4500000e+02 2.4445713e+00 2.3544034e+00 2.1235302e+00;
 6.5000000e+02 2.4759707e+00 2.3272594e+00 2.1095399e+00;
 6.5500000e+02 2.4972171e+00 2.3740163e+00 2.2784375e+00;
 6.6000000e+02 2.5097849e+00 2.3860963e+00 2.2428651e+00;
 6.6500000e+02 2.5065477e+00 2.3997949e+00 2.2799510e+00;
 6.7000000e+02 2.5730632e+00 2.4290040e+00 2.2868850e+00;
 6.7500000e+02 2.5946722e+00 2.5113439e+00 2.3464026e+00;
 6.8000000e+02 2.6176085e+00 2.4691387e+00 2.3852079e+00;
 6.8500000e+02 2.6617975e+00 2.5238041e+00 2.3312045e+00;
 6.9000000e+02 2.6564191e+00 2.5660257e+00 2.3319234e+00;
 6.9500000e+02 2.6419873e+00 2.5244299e+00 2.4381298e+00;
 7.0000000e+02 2.6854381e+00 2.5304905e+00 2.5072026e+00;
 7.0500000e+02 2.7714441e+00 2.6478322e+00 2.4316573e+00;
 7.1000000e+02 2.8117062e+00 2.6576801e+00 2.4334780e+00;
 7.1500000e+02 2.7711360e+00 2.6459799e+00 2.5008564e+00;
 7.2000000e+02 2.7695393e+00 2.7388331e+00 2.4733557e+00;
 7.2500000e+02 2.8093032e+00 2.7316285e+00 2.5036557e+00;
 7.3000000e+02 2.9190577e+00 2.7281264e+00 2.5858057e+00;
 7.3500000e+02 2.9425813e+00 2.7526357e+00 2.5737739e+00;
 7.4000000e+02 2.9646149e+00 2.7761468e+00 2.6277102e+00;
 7.4500000e+02 2.9997074e+00 2.7782833e+00 2.6104910e+00;
 7.5000000e+02 2.9864209e+00 2.9590428e+00 2.6783063e+00;
 7.5500000e+02 3.0217403e+00 2.7830755e+00 2.6928250e+00;
 7.6000000e+02 3.0724579e+00 2.9123083e+00 2.7241858e+00;
 7.6500000e+02 3.1086710e+00 2.9279470e+00 2.7749194e+00;
 7.7000000e+02 3.2031659e+00 2.9024787e+00 2.7873552e+00;
 7.7500000e+02 3.0878064e+00 2.9750201e+00 2.7151869e+00;
 7.8000000e+02 3.1599427e+00 3.0234076e+00 2.8747921e+00;
 7.8500000e+02 3.2016387e+00 3.0759631e+00 2.7766015e+00;
 7.9000000e+02 3.1648603e+00 3.0659770e+00 2.7805582e+00;
 7.9500000e+02 3.1800482e+00 3.1119445e+00 2.9145966e+00;
 8.0000000e+02 3.2759257e+00 3.1863498e+00 2.8699613e+00;
 8.0500000e+02 3.2692005e+00 3.1384007e+00 2.8504185e+00;
 8.1000000e+02 3.3268474e+00 3.1469754e+00 2.9498752e+00;
 8.1500000e+02 3.3168393e+00 3.1398081e+00 3.0506458e+00;
 8.2000000e+02 3.3808611e+00 3.2593534e+00 3.0382036e+00;
 8.2500000e+02 3.4181333e+00 3.2320960e+00 3.0555676e+00;
 8.3000000e+02 3.3526291e+00 3.3378267e+00 3.0105086e+00;
 8.3500000e+02 3.5036003e+00 3.3110840e+00 3.0466612e+00;
 8.4000000e+02 3.4197251e+00 3.3306372e+00 3.1416978e+00;
 8.4500000e+02 3.5420700e+00 3.3413187e+00 3.1837068e+00;
 8.5000000e+02 3.5621956e+00 3.3713473e+00 3.2010923e+00;
 8.5500000e+02 3.6005529e+00 3.3693904e+00 3.2047538e+00;
 8.6000000e+02 3.6832469e+00 3.4728465e+00 3.1676337e+00;
 8.6500000e+02 3.5879701e+00 3.4600635e+00 3.2272849e+00;
 8.7000000e+02 3.6529343e+00 3.5313021e+00 3.3103463e+00;
 8.7500000e+02 3.6800483e+00 3.4658495e+00 3.3439466e+00;
 8.8000000e+02 3.8048883e+00 3.5766463e+00 3.3299177e+00;
 8.8500000e+02 3.8256850e+00 3.5546218e+00 3.3448211e+00;
 8.9000000e+02 3.8313159e+00 3.5572564e+00 3.3809257e+00;
 8.9500000e+02 3.7693385e+00 3.6398592e+00 3.3748248e+00;
 9.0000000e+02 3.8448725e+00 3.7032433e+00 3.4549903e+00;
 9.0500000e+02 3.9159888e+00 3.7491897e+00 3.4566418e+00;
 9.1000000e+02 3.9398317e+00 3.7865530e+00 3.4934523e+00;
 9.1500000e+02 3.9298824e+00 3.7508924e+00 3.4956659e+00;
 9.2000000e+02 4.0492033e+00 3.7569062e+00 3.5415600e+00;
 9.2500000e+02 3.9951859e+00 3.9474937e+00 3.5311767e+00;
 9.3000000e+02 4.0659343e+00 3.8326208e+00 3.6478648e+00;
 9.3500000e+02 4.1633054e+00 3.8561042e+00 3.5995144e+00;
 9.4000000e+02 4.1839674e+00 3.9334921e+00 3.6665231e+00;
 9.4500000e+02 4.1366737e+00 3.9009719e+00 3.5875770e+00;
 9.5000000e+02 4.2073266e+00 3.9474348e+00 3.7333023e+00;
 9.5500000e+02 4.2770770e+00 3.9429081e+00 3.7064543e+00;
 9.6000000e+02 4.2013391e+00 4.0266060e+00 3.7738715e+00;
 9.6500000e+02 4.3116974e+00 4.0196141e+00 3.7598150e+00;
 9.7000000e+02 4.3807813e+00 4.1110343e+00 3.8322796e+00;
 9.7500000e+02 4.3779508e+00 4.1334772e+00 3.8116287e+00;
 9.8000000e+02 4.4196780e+00 4.1275399e+00 3.8439161e+00;
 9.8500000e+02 4.4015677e+00 4.1717386e+00 3.8507165e+00;
 9.9000000e+02 4.3815414e+00 4.1949918e+00 3.8575296e+00;
 9.9500000e+02 4.4370445e+00 4.2556787e+00 3.9632262e+00;
 1.0000000e+03 4.5629051e+00 4.2534913e+00 4.0190932e+00;
 1.0050000e+03 4.5353495e+00 4.3218812e+00 4.0096531e+00;
 1.0100000e+03 4.5992413e+00 4.3080067e+00 4.0056142e+00;
 1.0150000e+03 4.5569539e+00 4.3970392e+00 4.0148016e+00;
 1.0200000e+03 4.5590663e+00 4.3981573e+00 4.0504362e+00;
 1.0250000e+03 4.5810653e+00 4.3538817e+00 4.1899300e+00;
 1.0300000e+03 4.7097650e+00 4.4755318e+00 4.1516340e+00;
 1.0350000e+03 4.6446833e+00 4.5375482e+00 4.1334187e+00;
 1.0400000e+03 4.8236555e+00 4.5821527e+00 4.1610760e+00;
 1.0450000e+03 4.8457768e+00 4.5723715e+00 4.2054082e+00;
 1.0500000e+03 4.8452803e+00 4.6545741e+00 4.2559532e+00;
 1.0550000e+03 4.9087589e+00 4.6549759e+00 4.3755750e+00;
 1.0600000e+03 4.8966159e+00 4.7439687e+00 4.3254659e+00;
 1.0650000e+03 4.8615836e+00 4.7022483e+00 4.3034173e+00;
 1.0700000e+03 4.9641911e+00 4.7431845e+00 4.3540005e+00;
 1.0750000e+03 4.9764416e+00 4.7738693e+00 4.3734111e+00;
 1.0800000e+03 5.0885959e+00 4.8112968e+00 4.4970507e+00;
 1.0850000e+03 5.0864543e+00 4.8549443e+00 4.5540186e+00;
 1.0900000e+03 5.1237060e+00 4.8970771e+00 4.5652457e+00;
 1.0950000e+03 5.1269271e+00 4.8474671e+00 4.5194580e+00;
 1.1000000e+03 5.1409824e+00 4.8680138e+00 4.6122598e+00];

%% Least squares method

poly_1 = polyfit(x(:,1),x(:,2),2);
y_1 = polyval(poly_1,x(:,1));

poly_2 = polyfit(x(:,1),x(:,3),2);
y_2 = polyval(poly_2,x(:,1));

poly_3 = polyfit(x(:,1),x(:,4),2);
y_3 = polyval(poly_3,x(:,1));

figure
%plot(x(:,1),x(:,2),'+')
hold on
%plot(x(:,1),x(:,3),'x')
hold on
%plot(x(:,1),x(:,4), 'o')

hold on
plot(x(:,1),y_1,'+')
hold on
plot(x(:,1),y_2,'x')
hold on
plot(x(:,1),y_3,'-')

error_1 = y_1 - x(:,2);
error_2 = y_2 - x(:,3);
error_3 = y_3 - x(:,4);

%% Surface Fit for the whole data
Voltage_surface = x(:,2:4); 
Pressure_surface = ones(267,3);
Pressure_surface(:,1) = x(:,1);
Pressure_surface(:,2) = x(:,1);
Pressure_surface(:,3) = x(:,1);

Temperature_surface = ones(267, 3);
Temperature_surface(:,1) = Temperature_surface(:,1)*-45;
Temperature_surface(:,2) = Temperature_surface(:,2)*25;
Temperature_surface(:,3) = Temperature_surface(:,3)*125;

surface_fit = fit( [Pressure_surface(:), Temperature_surface(:)], Voltage_surface(:), 'poly21' );
y_1_surface = surface_fit(Pressure_surface(:,1),Temperature_surface(:,1));
y_2_surface = surface_fit(Pressure_surface(:,2),Temperature_surface(:,2));
y_3_surface = surface_fit(Pressure_surface(:,3),Temperature_surface(:,3));

error_surface_1 = y_1_surface - Voltage_surface(:,1);
error_surface_2 = y_2_surface - Voltage_surface(:,2);
error_surface_3 = y_3_surface - Voltage_surface(:,3);

figure
plot(surface_fit, [Pressure_surface(:),Temperature_surface(:)], Voltage_surface(:))

%% Inverse function
inv_poly_1 = polyfit(x(:,2),x(:,1),2);
inv_y_1 = polyval(inv_poly_1,x(:,2));

inv_poly_2 = polyfit(x(:,3),x(:,1),2);
inv_y_2 = polyval(inv_poly_2,x(:,3));

inv_poly_3 = polyfit(x(:,4),x(:,1),2);
inv_y_3 = polyval(inv_poly_3,x(:,4));

inv_error_1 = inv_y_1 - x(:,1);
inv_error_2 = inv_y_2 - x(:,1);
inv_error_3 = inv_y_3 - x(:,1);

%% Inverse Surface
inv_surface_fit = fit( [Voltage_surface(:), Temperature_surface(:)], Pressure_surface(:), 'poly21' );
inv_y_1_surface = inv_surface_fit(Voltage_surface(:,1),Temperature_surface(:,1));
inv_y_2_surface = inv_surface_fit(Voltage_surface(:,2),Temperature_surface(:,2));
inv_y_3_surface = inv_surface_fit(Voltage_surface(:,3),Temperature_surface(:,3));

inv_error_surface_1 = inv_y_1_surface - Pressure_surface(:,1);
inv_error_surface_2 = inv_y_2_surface - Pressure_surface(:,2);
inv_error_surface_3 = inv_y_3_surface - Pressure_surface(:,3);

%% Conversion to binary
% float2bin(0.1) =
% '0.0001100110011001100110011001100110011001100110011001101'   isto �
% demais para o arinc429 que s� tem 32 bits

%% Pressure Altitude
pa_data = 145366.45.*(1-(x(:,1)./1013.25).^0.190284);
pa_model_1 = 145366.45.*(1-(inv_y_1./1013.25).^0.190284);
pa_model_2 = 145366.45.*(1-(inv_y_2./1013.25).^0.190284);
pa_model_3 = 145366.45.*(1-(inv_y_3./1013.25).^0.190284);

pa_error_1 = pa_data - pa_model_1;
pa_error_2 = pa_data - pa_model_2;
pa_error_3 = pa_data - pa_model_3;

pa_model_1_surface = 145366.45.*(1-(inv_y_1_surface./1013.25).^0.190284);
pa_model_2_surface = 145366.45.*(1-(inv_y_2_surface./1013.25).^0.190284);
pa_model_3_surface = 145366.45.*(1-(inv_y_3_surface./1013.25).^0.190284);

pa_error_1_surface = pa_data - pa_model_1_surface;
pa_error_2_surface = pa_data - pa_model_2_surface;
pa_error_3_surface = pa_data - pa_model_3_surface;
