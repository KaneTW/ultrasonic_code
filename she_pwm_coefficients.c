#include "she_pwm_coefficients.h"

const float dutyPeriods[AMPLITUDES][ANGLES * 2] = {
  { 0.002617960232633609, 0.5226405006388023, 0.004534510064826858,
    0.5232509778411056, 0.005236041589917750, 0.5239525093661965,
    0.004534510064826858, 0.5245570504709955, 0.002617960232633720,
    1.047191615272693 },
  { 0.005235718559651770, 0.5216820891603372, 0.009069090046235573,
    0.5229089917055556, 0.01047240626971568, 0.5243123079290357,
    0.009069090046235573, 0.5255154606469210, 0.005235718559651659,
    1.047173804147944 },
  { 0.007853072908024505, 0.5207234036250851, 0.01360380981637221,
    0.5225726969641842, 0.01570941756094602, 0.5246783047087580,
    0.01360380981637199, 0.5264741405334328, 0.007853072908024394,
    1.047144107758333 },
  { 0.01046982086958215, 0.5197643053220589, 0.01813873915809983,
    0.5222419773255635, 0.02094739984999716, 0.5250506380174609,
    0.01813873915809960, 0.5274332236105765, 0.01046982086958215,
    1.047102509314133 },
  { 0.01308575953266178, 0.5188046542956145, 0.02267394771929454,
    0.5219167202158452, 0.02618667882424530, 0.5254294513207960,
    0.02267394771929476, 0.5283928424822473, 0.01308575953266189,
    1.047048985275290 },
  { 0.01570068531281910, 0.5178443092699688, 0.02720950496587404,
    0.5215968165665394, 0.03142758191271877, 0.5258148935133842,
    0.02720950496587404, 0.5293531289230238, 0.01570068531281921,
    1.046983505316877 },
  { 0.01831439378226773, 0.5168831275697255, 0.03174548013310496,
    0.5212821606076592, 0.03667043873281450, 0.5262071192073687,
    0.03174548013310474, 0.5303142139205628, 0.01831439378226785,
    1.046906032284477 },
  { 0.02092667949774618, 0.5159209650363451, 0.03628194217474134,
    0.5209726496653937, 0.04191558154465991, 0.5266062890353123,
    0.03628194217474112, 0.5312762277133403, 0.02092667949774629,
    1.046816522139402 },
  { 0.02353733582646911, 0.5149576759405222, 0.04081895970950766,
    0.5206681839634657, 0.04716334571474556, 0.5270125699687036,
    0.04081895970950766, 0.5322392998235608, 0.02353733582646900,
    1.046714923893541 },
  { 0.02614615476982152, 0.5139931128903689, 0.04535660096444594,
    0.5203686664273426, 0.05241407019053090, 0.5274261356534276,
    0.04535660096444571, 0.5332035590849933, 0.02614615476982163,
    1.046601179533661 },
  { 0.02875292678447128, 0.5130271267353168, 0.04989493371458953,
    0.5200740024904611, 0.05766809798777839, 0.5278471667636500,
    0.04989493371458931, 0.5341691336654351, 0.02875292678447128,
    1.046475223934930 },
  { 0.03135744060052081, 0.5120595664656180, 0.05443402521841789,
    0.5197840999016163, 0.06292577669246890, 0.5282758513756673,
    0.05443402521841767, 0.5351361510835151, 0.03135744060052081,
    1.046336984763376 },
  { 0.03395948303635410, 0.5110902791073020, 0.05897394214849938,
    0.5194988685326529, 0.06818745897923817, 0.5287123853633917,
    0.05897394214849916, 0.5361047382194473, 0.03395948303635388,
    1.046186382366999 },
  { 0.03655883880978916, 0.5101191096124394, 0.06351475051668776,
    0.5192182201855700, 0.07345350314838495, 0.5291569728172671,
    0.06351475051668798, 0.5370750213193380, 0.03655883880978905,
    1.046023329655179 },
  { 0.03915529034514731, 0.5091459007445263, 0.06805651559319492,
    0.5189420683981270, 0.07872427368363066, 0.5296098264885627,
    0.06805651559319470, 0.5380471259925739, 0.03915529034514709,
    1.045847731966004 },
  { 0.04174861757583259, 0.5081704929587973, 0.07259930181880714,
    0.5186703282469987, 0.08400014183294324, 0.5300711682611348,
    0.07259930181880714, 0.5390211772017719, 0.04174861757583237,
    1.045659486921091 },
  { 0.04433859774199655, 0.5071927242772445, 0.07714317270944604,
    0.5184029161474799, 0.08928148621490628, 0.5305412296529401,
    0.07714317270944582, 0.5399972992446940, 0.04433859774199655,
    1.045458484267435 },
  { 0.04692500518284659, 0.5062124301581022, 0.08168819075221734,
    0.5181397496486970, 0.09456869345328300, 0.5310202523497627,
    0.08168819075221734, 0.5409756157274730, 0.04692500518284648,
    1.045244605705758 },
  { 0.04950761112312230, 0.5052294433595295, 0.08623441729199777,
    0.5178807472232108, 0.09986215884263627, 0.5315084887738493,
    0.08623441729199754, 0.5419562495284049, 0.04950761112312208,
    1.045017724704799 },
  { 0.05208618345326793, 0.5042435937972021, 0.09078191240754352,
    0.5176258280498351, 0.1051622870480733, 0.5320062026903649,
    0.09078191240754352, 0.5429393227514776, 0.05208618345326776,
    1.044777706300914 },
  { 0.05466048650276845, 0.5032547083954972, 0.09533073477598175,
    0.5173749117884006, 0.1104694928424537, 0.5325136698548725,
    0.09533073477598153, 0.5439249566687105, 0.05466048650276845,
    1.044524406882312 },
  { 0.05723028080611942, 0.5022626109319266, 0.09988094152446558,
    0.5171279183451113, 0.1157842018846598, 0.5330311787053055,
    0.09988094152446569, 0.5449132716502727, 0.05723028080611936,
    1.044257673957177 },
  { 0.05979532286084538, 0.5012671218744444, 0.1044325880676154,
    0.5168847676270185, 0.1211068515428599, 0.5335590311022631,
    0.1044325880676151, 0.5459043870812144, 0.05979532286084543,
    1.043977345904853 },
  { 0.06235536487696958, 0.5002680582112264, 0.1089857279292691,
    0.5166453792840325, 0.1264378917670208, 0.5340975431217843,
    0.1089857279292690, 0.5468984212635259, 0.06235536487696969,
    1.043683251709224 },
  { 0.06491015451727528, 0.4992652332724760, 0.1135404125468792,
    0.5164096724367365, 0.1317777860153364, 0.5346470459051937,
    0.1135404125468793, 0.5478954913020799, 0.06491015451727522,
    1.043375210673307 },
  { 0.06745943462768067, 0.4982584565437851, 0.1180966910567331,
    0.5161775653881144, 0.1371270122396666, 0.5352078865710479,
    0.1180966910567331, 0.5488957129728376, 0.06745943462768089,
    1.043053032114008 },
  { 0.07000294295698667, 0.4972475334705314, 0.1226546100579839,
    0.5159489753171336, 0.1424860639355572, 0.5357804291947068,
    0.1226546100579839, 0.5498992005715286, 0.07000294295698684,
    1.042716515035893 },
  { 0.07254041186522947, 0.4962322652527582, 0.1272142133532392,
    0.5157238179519059, 0.1478554512629753, 0.5363650558616420,
    0.1272142133532390, 0.5509060667407679, 0.07254041186522953,
    1.042365447782719 },
  { 0.07507156801977127, 0.4952124486299178, 0.1317755416632380,
    0.5155020072199372, 0.1532357022444861, 0.5369621678011853,
    0.1317755416632380, 0.5519164222733846, 0.07507156801977111,
    1.041999607665368 },
  { 0.07759613207826799, 0.4941878756548361, 0.1363386323128420,
    0.5152834548727078, 0.1586273640482929, 0.5375721866081586,
    0.1363386323128419, 0.5529303758894102, 0.07759613207826810,
    1.041618760564680 },
  { 0.08011381835751696, 0.4931583334561721, 0.1409035188852660,
    0.5150680700815350, 0.1640310043643183, 0.5381955555605873,
    0.1409035188852661, 0.5539480339839211, 0.08011381835751674,
    1.041222660507578 },
  { 0.08262433448716877, 0.4921236039886030, 0.1454702308411183,
    0.5148557590013426, 0.1694472128823765, 0.5388327410426008,
    0.1454702308411182, 0.5549695003425525, 0.08262433448716866,
    1.040811049214694 },
  { 0.08512738104718287, 0.4910834637698952, 0.1500387930984256,
    0.5146464242985817, 0.1748766028824456, 0.5394842340826016,
    0.1500387930984255, 0.5559948758211379, 0.08512738104718309,
    1.040383655617577 },
  { 0.08762265118780954, 0.4900376836039352, 0.1546092255693564,
    0.5144399646391310, 0.1803198129481438, 0.5401505520179184,
    0.1546092255693563, 0.5570242579854821, 0.08762265118780954,
    1.039940195343326 },
  { 0.09010983023080910, 0.4889860282887213, 0.1591815426488600,
    0.5142362741315168, 0.1857775088157481, 0.5408322402984049,
    0.1591815426488599, 0.5580577407067722, 0.09010983023080898,
    1.039480370164378 },
  { 0.09258859525048524, 0.4879282563082363, 0.1637557526498425,
    0.5140352417202478, 0.1912503853724732, 0.5415298744428785,
    0.1637557526498425, 0.5590954137075936, 0.09258859525048546,
    1.039003867410837 },
  { 0.09505861463301468, 0.4868641195069960, 0.1683318571788589,
    0.5138367505234452, 0.1967391688193123, 0.5422440621638986,
    0.1683318571788588, 0.5601373620528403, 0.09505861463301457,
    1.038510359342613 },
  { 0.09751954761238224, 0.4857933627459760, 0.1729098504455374,
    0.5136406771082321, 0.2022446190155196, 0.5429754456782143,
    0.1729098504455373, 0.5611836655791311, 0.09751954761238224,
    1.037999502478240 },
  { 0.09997104378112798, 0.4847157235384745, 0.1774897184981200,
    0.5134468906965565, 0.2077675320238299, 0.5437247042222664,
    0.1774897184981201, 0.5622343982554665, 0.09997104378112809,
    1.037470936877029 },
  { 0.1024127425739211, 0.4836309316643662, 0.1820714383764996,
    0.5132552522931804, 0.2133087428778233, 0.5444925567945040,
    0.1820714383764996, 0.5632896274669448, 0.1024127425739212,
    1.036924285370798 },
  { 0.1048442727217776, 0.4825387087609768, 0.1866549771730559,
    0.5130656137265459, 0.2188691285954532, 0.5452797651489432,
    0.1866549771730557, 0.5643494132122551, 0.1048442727217775,
    1.036359152741072 },
  { 0.1072652516745690, 0.4814387678887215, 0.1912402909902960,
    0.5128778165919979, 0.2244496114657504, 0.5460871370674523,
    0.1912402909902959, 0.5654138072044486, 0.1072652516745691,
    1.035775124837173 },
  { 0.1096752849891954, 0.4803308130693505, 0.1958273237828549,
    0.5126916910854880, 0.2300511626391355, 0.5469155299417685,
    0.1958273237828547, 0.5664828518630101, 0.1096752849891955,
    1.035171767630176 },
  { 0.1120739656805596, 0.4792145387945160, 0.2004160060697310,
    0.5125070547142766, 0.2356748060556888, 0.5477658547002344,
    0.2004160060697311, 0.5675565791836874, 0.1120739656805596,
    1.034548626197079 },
  { 0.1144608735321704, 0.4780896295020317, 0.2050062535007036,
    0.5123237108693510, 0.2413216227502279, 0.5486390801188754,
    0.2050062535007036, 0.5686350094705648, 0.1144608735321704,
    1.033905223628970 },
  { 0.1168355743628686, 0.4769557590169767, 0.2095979652586387,
    0.5121414472421409, 0.2469927555782201, 0.5495362375617223,
    0.2095979652586386, 0.5697181499127468, 0.1168355743628688,
    1.033241059856206 },
  { 0.1191976192457950, 0.4758125899544107, 0.2141910222768302,
    0.5119600340657091, 0.2526894144125276, 0.5504584262014065,
    0.2141910222768302, 0.5708059929854459, 0.1191976192457949,
    1.032555610382821 },
  { 0.1215465436752705, 0.4746597730801159, 0.2187852852475182,
    0.5117792221577587, 0.2584128818678866, 0.5514068187781271,
    0.2187852852475183, 0.5718985146523636, 0.1215465436752705,
    1.031848324921428 },
  { 0.1238818666767798, 0.4734969466253360, 0.2233805923942700,
    0.5115987407395259, 0.2641645196180225, 0.5523826679632784,
    0.2233805923942700, 0.5729956723428262, 0.1238818666767800,
    1.031118625918826 },
  { 0.1262030898546717, 0.4723237355509739, 0.2279767569768665,
    0.5114182950008317, 0.2699457753795906, 0.5533873134035558,
    0.2279767569768665, 0.5740974026731687, 0.1262030898546715,
    1.030365906961263 },
  { 0.1285096963715377, 0.4711397507561371, 0.2325735644926316,
    0.5112375633771112, 0.2757581906479589, 0.5544221895324386,
    0.2325735644926317, 0.5752036188772309, 0.1285096963715375,
    1.029589531046875 },
  { 0.1308011498524807, 0.4699445882252398, 0.2371707695326174,
    0.5110561944990563, 0.2816034092824751, 0.5554888342489139,
    0.2371707695326175, 0.5763142079053765, 0.1308011498524806,
    1.028788828711207 },
  { 0.1330768932066140, 0.4687378281070790, 0.2417680922446149,
    0.5108738037694117, 0.2874831870536734, 0.5565888985784702,
    0.2417680922446150, 0.5774290271450799, 0.1330768932066140,
    1.027963095989752 },
  { 0.1353363473571177, 0.4675190337183964, 0.2463652143473385,
    0.5106899695143203, 0.2933994022822279, 0.5577241574492098,
    0.2463652143473387, 0.5785479007086172, 0.1353363473571179,
    1.027111592199249 },
  { 0.1375789098699989, 0.4662877504633499, 0.2509617746311800,
    0.5105042286481646, 0.2993540677199444, 0.5588965217369291,
    0.2509617746311799, 0.5796706152245310, 0.1375789098699989,
    1.026233537516819 },
  { 0.1398039534703187, 0.4650435046590441, 0.2555573638703237,
    0.5103160717809019, 0.3053493438472366, 0.5601080517578148,
    0.2555573638703235, 0.5807969150590491, 0.1398039534703188,
    1.025328110332983 },
  { 0.1420108244330264, 0.4637858022557720, 0.2601515190584286,
    0.5101249376850540, 0.3113875537901936, 0.5613609724168189,
    0.2601515190584287, 0.5819264968811741, 0.1420108244330263,
    1.024394444350974 },
  { 0.1441988408336199, 0.4625141274388411, 0.2647437168651201,
    0.5099302070254643, 0.3174712000944071, 0.5626576902547513,
    0.2647437168651203, 0.5830590034703413, 0.1441988408336199,
    1.023431625400395 },
  { 0.1463672906415702, 0.4612279410966988, 0.2693333661926500,
    0.5097311952381622, 0.3236029836333643, 0.5640008126788765,
    0.2693333661926498, 0.5841940166477786, 0.1463672906415701,
    1.022438687928277 },
  { 0.1485154296367359, 0.4599266791375441, 0.2739197996906720,
    0.5095271444245845, 0.3297858249779004, 0.5653931697118129,
    0.2739197996906721, 0.5853310491914803, 0.1485154296367357,
    1.021414611124372 },
  { 0.1506424791257312, 0.4586097506335019, 0.2785022640613205,
    0.5093172141032897, 0.3360228886116778, 0.5668378386536470,
    0.2785022640613206, 0.5864695355690912, 0.1506424791257310,
    1.020358314630263 },
  { 0.1527476234313072, 0.4572765357677295, 0.2830799089556901,
    0.5091004706321824, 0.3423176104482750, 0.5683381721247673,
    0.2830799089556901, 0.5876088212921124, 0.1527476234313072,
    1.019268653773001 },
  { 0.1548300071230539, 0.4559263835552635, 0.2876517742251554,
    0.5088758750790545, 0.3486737291909541, 0.5698978300448532,
    0.2876517742251554, 0.5887481506573651, 0.1548300071230537,
    1.018144414253257 },
  { 0.1568887319519565, 0.4545586093028564, 0.2922167752451363,
    0.5086422692754260, 0.3550953221802464, 0.5715208162105361,
    0.2922167752451363, 0.5898866525960362, 0.1568887319519563,
    1.016984306204939 },
  { 0.1589228534442939, 0.4531724917662329, 0.2967736859729193,
    0.5083983597363912, 0.3615868465016416, 0.5732115202651136,
    0.2967736859729193, 0.5910233242948584, 0.1589228534442939,
    1.015786957527197 },
  { 0.1609313771016477, 0.4517672699546951, 0.3013211193324303,
    0.5081426990651177, 0.3681531862817771, 0.5749747660144644,
    0.3013211193324303, 0.5921570121854778, 0.1609313771016478,
    1.014550906370038 },
  { 0.1629132541430480, 0.4503421395225177, 0.3058575044340900,
    0.5078736643816838, 0.3747997072941587, 0.5768158672417525,
    0.3058575044340901, 0.5932863898135596, 0.1629132541430480,
    1.013274592630280 },
  { 0.1648673767118385, 0.4488962486733398, 0.3103810600328850,
    0.5075894322182749, 0.3815323202342835, 0.5787406924196734,
    0.3103810600328849, 0.5944099319943863, 0.1648673767118387,
    1.011956348284119 },
  { 0.1667925724529909, 0.4474286934871358, 0.3148897634969673,
    0.5072879492012151, 0.3883575543218365, 0.5807557400260843,
    0.3148897634969674, 0.5955258845311122, 0.1667925724529908,
    1.010594386344246 },
  { 0.1686875983453357, 0.4459385125582540, 0.3193813143952546,
    0.5069668976882982, 0.3952826432610159, 0.5828682265540595,
    0.3193813143952546, 0.5966322286081729, 0.1686875983453358,
    1.009186788181009 },
  { 0.1705511336460437, 0.4444246808059972, 0.3238530916061314,
    0.5066236553387077, 0.4023156260609464, 0.5850861897935227,
    0.3238530916061315, 0.5977266387660849, 0.1705511336460437,
    1.007731488885481 },
  { 0.1723817717699584, 0.4428861022845121, 0.3283021025878149,
    0.5062552473509042, 0.4094654658158179, 0.5874186105789072,
    0.3283021025878148, 0.5988064331023686, 0.1723817717699583,
    1.006226260273101 },
  { 0.1741780108814253, 0.4413216017735987, 0.3327249231173867,
    0.5058582897958747, 0.4167421903079687, 0.5898755569864567,
    0.3327249231173868, 0.5998685140095601, 0.1741780108814250,
    1.004668691024303 },
  { 0.1759382429176727, 0.4397299148731298, 0.3371176253771561,
    0.5054289220783760, 0.4241570592794872, 0.5924683559807071,
    0.3371176253771562, 0.6009092973326131, 0.1759382429176726,
    1.003056163324967 },
  { 0.1776607406857308, 0.4381096762458380, 0.3414756917131043,
    0.5049627260493474, 0.4317227644911350, 0.5952097988273781,
    0.3414756917131041, 0.6019246272732115, 0.1776607406857309,
    1.001385825194018 },
  { 0.1793436425725051, 0.4364594055495093, 0.3457939106683225,
    0.5044546286302096, 0.4394536703506513, 0.5981143883125384,
    0.3457939106683223, 0.6029096736453268, 0.1793436425725052,
    0.9996545574522089 },
  { 0.1809849342704048, 0.4347774904601401, 0.3500662509460847,
    0.5038987839416315, 0.4473661050833750, 0.6011986380789218,
    0.3500662509460846, 0.6038588071358200, 0.1809849342704046,
    0.9978589339732800 },
  { 0.1825824267348741, 0.4330621659979487, 0.3542857077008151,
    0.5032884297821333, 0.4554787153299258, 0.6044814374112439,
    0.3542857077008150, 0.6047654469638897, 0.1825824267348741,
    0.9959951734345776 },
  { 0.1841337293352960, 0.4313114891076393, 0.3584441138751896,
    0.5026157117723510, 0.4638129009624015, 0.6079844988595628,
    0.3584441138751895, 0.6056218736475329, 0.1841337293352958,
    0.9940590802027072 },
  { 0.1856362168070504, 0.4295233070818896, 0.3625319070332063,
    0.5018714664221817, 0.4723933522057573, 0.6117329115947328,
    0.3625319070332063, 0.6064189973080454, 0.1856362168070502,
    0.9920459711829439 },
  { 0.1870869881157433, 0.4276952179068452, 0.3665378390438940,
    0.5010449515791955, 0.4812487184066931, 0.6157558309419946,
    0.3665378390438938, 0.6071460688349959, 0.1870869881157433,
    0.9899505843267620 },
  { 0.1884828146367778, 0.4258245198797343, 0.3704486116996086,
    0.5001235088703093, 0.4904124478515315, 0.6200873450222322,
    0.3704486116996089, 0.6077903169425651, 0.1884828146367776,
    0.9877669628749524 },
  { 0.1898200740300310, 0.4239081467929501, 0.3742484153885921,
    0.4990921374012380, 0.4999238521575684, 0.6247675741702143,
    0.3742484153885921, 0.6083364881515112, 0.1898200740300311,
    0.9854883070738794 },
  { 0.1910946646853083, 0.4219425834241561, 0.3779183395033473,
    0.4979329504513822, 0.5098294688634928, 0.6298440798115277,
    0.3779183395033472, 0.6087662582421951, 0.1910946646853082,
    0.9831067816605321 },
  { 0.1923018933646964, 0.4199237537419188, 0.3814356111597860,
    0.4966244761656412, 0.5201848248810914, 0.6353736898869466,
    0.3814356111597859, 0.6090574715370083, 0.1923018933646965,
    0.9806132622582782 },
  { 0.1934363252378599, 0.4178468706770002, 0.3847726011665911,
    0.4951407477105985, 0.5310567460960067, 0.6414248926400141,
    0.3847726011665911, 0.6091831466057314, 0.1934363252378599,
    0.9779969959564490 },
  { 0.1944915801645719, 0.4157062307561056, 0.3878955100684937,
    0.4934501055390977, 0.5425264220909396, 0.6480810175615437,
    0.3878955100684935, 0.6091101606600273, 0.1944915801645717,
    0.9752451390730190 },
  { 0.1954600505631380, 0.4134949280268566, 0.3907626077143256,
    0.4915136003160503, 0.5546935319710125, 0.6554445245727372,
    0.3907626077143254, 0.6087974851780442, 0.1954600505631379,
    0.9723421154961049 },
  { 0.1963325022711233, 0.4112044471700035, 0.3933218392713465,
    0.4892828332285195, 0.5676818882086430, 0.6636428821658160,
    0.3933218392713462, 0.6081937841702267, 0.1963325022711233,
    0.9692687068552273 },
  { 0.1970974963391447, 0.4088240711809732, 0.3955075155488975,
    0.4866969901032275, 0.5816472958725596, 0.6728367704268896,
    0.3955075155488976, 0.6072340903907261, 0.1970974963391448,
    0.9660007314879766 },
  { 0.1977405288619269, 0.4063399962844221, 0.3972356526960572,
    0.4836786988405894, 0.5967887178059623, 0.6832317639504946,
    0.3972356526960570, 0.6058351201185523, 0.1977405288619267,
    0.9625070743957349 },
  { 0.1982427121951388, 0.4037339695070327, 0.3983972744181892,
    0.4801281349001392, 0.6133644984167423, 0.6950953588986923,
    0.3983972744181892, 0.6038885317300831, 0.1982427121951389,
    0.9587466585538458 },
  { 0.1985786818997354, 0.4009811186621104, 0.3988485629425146,
    0.4759144623277164, 0.6317165509301823, 0.7087824503153841,
    0.3988485629425147, 0.6012509997048896, 0.1985786818997353,
    0.9546636225796927 },
  { 0.1987131385163587, 0.3980463560461393, 0.3983959999493378,
    0.4708631321210297, 0.6523074939781939, 0.7247746261498857,
    0.3983959999493378, 0.5977292174791184, 0.1987131385163585,
    0.9501793217936201 },
  { 0.1985948557848101, 0.3948781325239759, 0.3967732986339301,
    0.4647365999715343, 0.6757796441592712, 0.7437429454968754,
    0.3967732986339303, 0.5930565753730959, 0.1985948557848101,
    0.9451784002243118 },
  { 0.1981456886748179, 0.3913969631149989, 0.3936044483766408,
    0.4572044105485267, 0.7030525217822725, 0.7666524839541584,
    0.3936044483766405, 0.5868557228168219, 0.1981456886748179,
    0.9394830731552872 },
  { 0.1972389408461032, 0.3874728504237692, 0.3883425192259817,
    0.4477960609564082, 0.7354917063551789, 0.7949452480856054,
    0.3883425192259815, 0.5785764288036477, 0.1972389408461033,
    0.9328020653203626 },
  { 0.1956528411546562, 0.3828768959722166, 0.3801652134468102,
    0.4358272703235110, 0.7752177593935881, 0.8308798162702888,
    0.3801652134468103, 0.5673892682643706, 0.1956528411546561,
    0.9246194027594061 },
  { 0.1929581938716038, 0.3771656284094717, 0.3677946537786203,
    0.4202951727635971, 0.8257086042613753, 0.8782091232463520,
    0.3677946537786201, 0.5520020883164882, 0.1929581938716036,
    0.9139206408538842 },
  { 0.1882005675665525, 0.3693624661908743, 0.3492118566603868,
    0.3998091136071873, 0.8930571770758426, 0.9436544340226432,
    0.3492118566603866, 0.5303737552847085, 0.1882005675665526,
    0.8983928844843798 }
};
