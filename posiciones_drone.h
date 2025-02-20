int posicion_drone_i = 0;

int n_posiciones = 300;

double posicion_drone[300][3] = {
    {-34.59612126, -58.48134331, 0.0},
    {-34.59612126, -58.48134331, 0.0},
    {-34.59612126, -58.48134331, 0.0},  // LAT, LON, ALT 1
    {-34.59587208, -58.48137107, 5.0},
    {-34.59587208, -58.48137107, 5.0},
    {-34.59587208, -58.48137107, 5.0}, // LAT, LON, ALT  2
    {-34.59564353, -58.48138958, 10.0},
    {-34.59564353, -58.48138958, 10.0},
    {-34.59564353, -58.48138958, 10.0}, // LAT, LON, ALT  3
    {-34.59540736, -58.48144511, 15.0},
    {-34.59540736, -58.48144511, 15.0},
    {-34.59540736, -58.48144511, 15.0},
    {-34.59515596, -58.48150989, 20.0},
    {-34.59515596, -58.48150989, 20.0},
    {-34.59515596, -58.48150989, 20.0},
    {-34.59489694, -58.48154691, 25.0},
    {-34.59489694, -58.48154691, 25.0},
    {-34.59489694, -58.48154691, 25.0},
    {-34.59469124, -58.48164871, 30.0},
    {-34.59469124, -58.48164871, 30.0},
    {-34.59469124, -58.48164871, 30.0},
    {-34.59445507, -58.48174126, 35.0},
    {-34.59445507, -58.48174126, 35.0},
    {-34.59445507, -58.48174126, 35.0},
    {-34.59419604, -58.48186157, 40.0},
    {-34.59419604, -58.48186157, 40.0},
    {-34.59419604, -58.48186157, 40.0},
    {-34.59390271, -58.48205162, 45.0},
    {-34.59390271, -58.48205162, 45.0},
    {-34.59390271, -58.48205162, 45.0},
    {-34.59362396, -58.48222031, 50.0},
    {-34.59362396, -58.48222031, 50.0},
    {-34.59362396, -58.48222031, 50.0},
    {-34.59330106, -58.48244158, 55.0},
    {-34.59330106, -58.48244158, 55.0},
    {-34.59330106, -58.48244158, 55.0},
    {-34.59293677, -58.48265278, 60.0},
    {-34.59293677, -58.48265278, 60.0},
    {-34.59293677, -58.48265278, 60.0},
    {-34.59269667, -58.48294445, 65.0},
    {-34.59269667, -58.48294445, 65.0},
    {-34.59269667, -58.48294445, 65.0},
    {-34.59230754, -58.48336686, 70.0},
    {-34.59230754, -58.48336686, 70.0},
    {-34.59230754, -58.48336686, 70.0},
    {-34.59200120, -58.48377921, 75.0},
    {-34.59200120, -58.48377921, 75.0},
    {-34.59200120, -58.48377921, 75.0},
    {-34.59180250, -58.48412117, 80.0},
    {-34.59180250, -58.48412117, 80.0},
    {-34.59180250, -58.48412117, 80.0},
    {-34.59157895, -58.48453352, 85.0},
    {-34.59157895, -58.48453352, 85.0},
    {-34.59157895, -58.48453352, 85.0},
    {-34.59146304, -58.48477490, 90.0},
    {-34.59146304, -58.48477490, 90.0},
    {-34.59146304, -58.48477490, 90.0},
    {-34.59128917, -58.48513697, 95.0},
    {-34.59128917, -58.48513697, 95.0},
    {-34.59128917, -58.48513697, 95.0},
    {-34.59112358, -58.48553926, 100.0},
    {-34.59112358, -58.48553926, 100.0},
    {-34.59112358, -58.48553926, 100.0},
    {-34.59098283, -58.48604213, 95.0},
    {-34.59098283, -58.48604213, 95.0},
    {-34.59098283, -58.48604213, 95.0},
    {-34.59088347, -58.48633380, 90.0},
    {-34.59088347, -58.48633380, 90.0},
    {-34.59088347, -58.48633380, 90.0},
    {-34.59082552, -58.48662546, 85.0},
    {-34.59082552, -58.48662546, 85.0},
    {-34.59082552, -58.48662546, 85.0},
    {-34.59077584, -58.48695736, 80.0},
    {-34.59077584, -58.48695736, 80.0},
    {-34.59077584, -58.48695736, 80.0},
    {-34.59074272, -58.48733954, 75.0},
    {-34.59074272, -58.48733954, 75.0},
    {-34.59074272, -58.48733954, 75.0},
    {-34.59069304, -58.48775189, 70.0},
    {-34.59069304, -58.48775189, 70.0},
    {-34.59069304, -58.48775189, 70.0},
    {-34.59065993, -58.48827488, 65.0},
    {-34.59065993, -58.48827488, 65.0},
    {-34.59065993, -58.48827488, 65.0},
    {-34.59063509, -58.48866712, 60.0},
    {-34.59063509, -58.48866712, 60.0},
    {-34.59063509, -58.48866712, 60.0},
    {-34.59069304, -58.48901913, 55.0},
    {-34.59069304, -58.48901913, 55.0},
    {-34.59069304, -58.48901913, 55.0},
    {-34.59072616, -58.48951194, 50.0},
    {-34.59072616, -58.48951194, 50.0},
    {-34.59072616, -58.48951194, 50.0},
    {-34.59078412, -58.48979355, 45.0},
    {-34.59078412, -58.48979355, 45.0},
    {-34.59078412, -58.48979355, 45.0},
    {-34.59085864, -58.49019585, 40.0},
    {-34.59085864, -58.49019585, 40.0},
    {-34.59085864, -58.49019585, 40.0},
    {-34.59093158, -58.49040226, 35.0},
    {-34.59093158, -58.49040226, 35.0},
    {-34.59093158, -58.49040226, 35.0},
    {-34.59105708, -58.49063093, 30.0},
    {-34.59105708, -58.49063093, 30.0},
    {-34.59105708, -58.49063093, 30.0},
    {-34.59117002, -58.49104252, 25.0},
    {-34.59117002, -58.49104252, 25.0},
    {-34.59117002, -58.49104252, 25.0},
    {-34.59122022, -58.49125594, 20.0},
    {-34.59122022, -58.49125594, 20.0},
    {-34.59122022, -58.49125594, 20.0},
    {-34.59142101, -58.49153033, 15.0},
    {-34.59142101, -58.49153033, 15.0},
    {-34.59142101, -58.49153033, 15.0},
    {-34.59154650, -58.49186571, 10.0},
    {-34.59154650, -58.49186571, 10.0},
    {-34.59154650, -58.49186571, 10.0},
    {-34.59169710, -58.49220108, 5.0},
    {-34.59169710, -58.49220108, 5.0},
    {-34.59169710, -58.49220108, 5.0},
    {-34.59188533, -58.49252121, 5.0},
    {-34.59188533, -58.49252121, 5.0},
    {-34.59188533, -58.49252121, 5.0},
    {-34.59206102, -58.49279560, 10.0},
    {-34.59206102, -58.49279560, 10.0},
    {-34.59206102, -58.49279560, 10.0},
    {-34.59218502, -58.49298130, 20.0},
    {-34.59218502, -58.49298130, 20.0},
    {-34.59218502, -58.49298130, 20.0},
    {-34.59237620, -58.49317373, 30.0},
    {-34.59237620, -58.49317373, 30.0},
    {-34.59237620, -58.49317373, 30.0},
    {-34.59254554, -58.49340597, 40.0},
    {-34.59254554, -58.49340597, 40.0},
    {-34.59254554, -58.49340597, 40.0},
    {-34.59269302, -58.49351213, 50.0},
    {-34.59269302, -58.49351213, 50.0},
    {-34.59269302, -58.49351213, 50.0},
    {-34.59286235, -58.49372447, 60.0},
    {-34.59286235, -58.49372447, 60.0},
    {-34.59286235, -58.49372447, 60.0},
    {-34.59306446, -58.49385718, 70.0},
    {-34.59306446, -58.49385718, 70.0},
    {-34.59306446, -58.49385718, 70.0},
    {-34.59322833, -58.49404297, 80.0},
    {-34.59322833, -58.49404297, 80.0},
    {-34.59322833, -58.49404297, 80.0},
    {-34.59332509, -58.49412917, 90.0},
    {-34.59332509, -58.49412917, 90.0},
    {-34.59332509, -58.49412917, 90.0},
    {-34.59352507, -58.49421537, 100.0},
    {-34.59352507, -58.49421537, 100.0},
    {-34.59352507, -58.49421537, 100.0},
    {-34.59373795, -58.49434859, 100.0},
    {-34.59373795, -58.49434859, 100.0},
    {-34.59373795, -58.49434859, 100.0},
    {-34.59389922, -58.49450531, 100.0},
    {-34.59389922, -58.49450531, 100.0},
    {-34.59389922, -58.49450531, 100.0},
    {-34.59407985, -58.49456801, 100.0},
    {-34.59407985, -58.49456801, 100.0},
    {-34.59407985, -58.49456801, 100.0},
    {-34.59419596, -58.49466204, 100.0},
    {-34.59419596, -58.49466204, 100.0},
    {-34.59419596, -58.49466204, 100.0},
    {-34.59441529, -58.49476392, 100.0},
    {-34.59441529, -58.49476392, 100.0},
    {-34.59441529, -58.49476392, 100.0},
    {-34.59464107, -58.49485795, 100.0},
    {-34.59464107, -58.49485795, 100.0},
    {-34.59464107, -58.49485795, 100.0},
    {-34.59492491, -58.49498333, 100.0},
    {-34.59492491, -58.49498333, 100.0},
    {-34.59492491, -58.49498333, 100.0},
    {-34.59521519, -58.49507737, 100.0},
    {-34.59521519, -58.49507737, 100.0},
    {-34.59521519, -58.49507737, 100.0},
    {-34.59553128, -58.49514790, 100.0},
    {-34.59553128, -58.49514790, 100.0},
    {-34.59553128, -58.49514790, 100.0},
    {-34.59581511, -58.49521059, 100.0},
    {-34.59581511, -58.49521059, 100.0},
    {-34.59581511, -58.49521059, 100.0},
    {-34.59604089, -58.49524194, 150.0},
    {-34.59604089, -58.49524194, 150.0},
    {-34.59604089, -58.49524194, 150.0},
    {-34.59632765, -58.49518646, 150.0},
    {-34.59632765, -58.49518646, 150.0},
    {-34.59632765, -58.49518646, 150.0},
    {-34.59659258, -58.49520657, 150.0},
    {-34.59659258, -58.49520657, 150.0},
    {-34.59659258, -58.49520657, 150.0},
    {-34.59688234, -58.49522669, 150.0},
    {-34.59688234, -58.49522669, 150.0},
    {-34.59688234, -58.49522669, 150.0},
    {-34.59718039, -58.49522669, 150.0},
    {-34.59718039, -58.49522669, 150.0},
    {-34.59718039, -58.49522669, 150.0},
    {-34.59743703, -58.49514623, 150.0},
    {-34.59743703, -58.49514623, 150.0},
    {-34.59743703, -58.49514623, 150.0},
    {-34.59787581, -58.49505571, 150.0},
    {-34.59787581, -58.49505571, 150.0},
    {-34.59787581, -58.49505571, 150.0},
    {-34.59824836, -58.49489479, 150.0},
    {-34.59824836, -58.49489479, 150.0},
    {-34.59824836, -58.49489479, 150.0},
    {-34.59856295, -58.49476405, 150.0},
    {-34.59856295, -58.49476405, 150.0},
    {-34.59856295, -58.49476405, 150.0},
    {-34.59892722, -58.49453272, 150.0},
    {-34.59892722, -58.49453272, 150.0},
    {-34.59892722, -58.49453272, 150.0},
    {-34.59909279, -58.49443215, 150.0},
    {-34.59909279, -58.49443215, 150.0},
    {-34.59909279, -58.49443215, 150.0},
    {-34.59925837, -58.49425112, 50.0},
    {-34.59925837, -58.49425112, 50.0},
    {-34.59925837, -58.49425112, 50.0},
    {-34.59966349, -58.49387458, 50.0},
    {-34.59966349, -58.49387458, 50.0},
    {-34.59966349, -58.49387458, 50.0},
    {-34.60023407, -58.49327383, 50.0},
    {-34.60023407, -58.49327383, 50.0},
    {-34.60023407, -58.49327383, 50.0},
    {-34.60072857, -58.49274239, 50.0},
    {-34.60072857, -58.49274239, 50.0},
    {-34.60072857, -58.49274239, 50.0},
    {-34.60128012, -58.49156399, 50.0},
    {-34.60128012, -58.49156399, 50.0},
    {-34.60128012, -58.49156399, 50.0},
    {-34.60158443, -58.49094013, 50.0},
    {-34.60158443, -58.49094013, 50.0},
    {-34.60158443, -58.49094013, 50.0},
    {-34.60171756, -58.49017764, 50.0},
    {-34.60171756, -58.49017764, 50.0},
    {-34.60171756, -58.49017764, 50.0},
    {-34.60200284, -58.48950757, 50.0},
    {-34.60200284, -58.48950757, 50.0},
    {-34.60200284, -58.48950757, 50.0},
    {-34.60217401, -58.48862955, 50.0},
    {-34.60217401, -58.48862955, 50.0},
    {-34.60217401, -58.48862955, 50.0},
    {-34.60219303, -58.48809811, 50.0},
    {-34.60219303, -58.48809811, 50.0},
    {-34.60219303, -58.48809811, 50.0},
    {-34.60217401, -58.48747425, 50.0},
    {-34.60217401, -58.48747425, 50.0},
    {-34.60217401, -58.48747425, 50.0},
    {-34.60202186, -58.48668865, 20.0},
    {-34.60202186, -58.48668865, 20.0},
    {-34.60202186, -58.48668865, 20.0},
    {-34.60185069, -58.48599547, 20.0},
    {-34.60185069, -58.48599547, 20.0},
    {-34.60185069, -58.48599547, 20.0},
    {-34.60152737, -58.48518677, 20.0},
    {-34.60152737, -58.48518677, 20.0},
    {-34.60152737, -58.48518677, 20.0},
    {-34.60120405, -58.48442428, 20.0},
    {-34.60120405, -58.48442428, 20.0},
    {-34.60120405, -58.48442428, 20.0},
    {-34.60065249, -58.48356936, 20.0},
    {-34.60065249, -58.48356936, 20.0},
    {-34.60065249, -58.48356936, 20.0},
    {-34.60036721, -58.48315345, 20.0},
    {-34.60036721, -58.48315345, 20.0},
    {-34.60036721, -58.48315345, 20.0},
    {-34.59972055, -58.48252959, 20.0},
    {-34.59972055, -58.48252959, 20.0},
    {-34.59972055, -58.48252959, 20.0},
    {-34.59911193, -58.48220611, 10.0},
    {-34.59911193, -58.48220611, 10.0},
    {-34.59911193, -58.48220611, 10.0},
    {-34.59867448, -58.48202126, 10.0},
    {-34.59867448, -58.48202126, 10.0},
    {-34.59867448, -58.48202126, 10.0},
    {-34.59817997, -58.48181331, 10.0},
    {-34.59817997, -58.48181331, 10.0},
    {-34.59817997, -58.48181331, 10.0},
    {-34.59770448, -58.48158225, 10.0},
    {-34.59770448, -58.48158225, 10.0},
    {-34.59770448, -58.48158225, 10.0},
    {-34.59717192, -58.48142051, 10.0},
    {-34.59717192, -58.48142051, 10.0},
    {-34.59717192, -58.48142051, 10.0},
    {-34.59677250, -58.48135119, 10.0},
    {-34.59677250, -58.48135119, 10.0},
    {-34.59677250, -58.48135119, 10.0},
    {-34.59633504, -58.48130498, 10.0},
    {-34.59633504, -58.48130498, 10.0},
    {-34.59633504, -58.48130498, 10.0},
    {-34.59596531, -58.48128276, 10.0},
    {-34.59596531, -58.48128276, 10.0},
    {-34.59596531, -58.48128276, 10.0},
    {-34.59573950, -58.48127906, 0.0},
    {-34.59573950, -58.48127906, 0.0},
    {-34.59573950, -58.48127906, 0.0},
    {-34.59658168, -58.48133466, 0.0},
    {-34.59658168, -58.48133466, 0.0},
    {-34.59658168, -58.48133466, 0.0}
};