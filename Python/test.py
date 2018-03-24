import json
from flock_control import *

response_raw = '{"generation":1,"birds":[{"position":{"x":2.7885546684265138,"y":36.345455169677737},"rectCorners":{"topLeft":{"x":1.6999046802520753,"y":36.780914306640628},"topRight":{"x":3.877204656600952,"y":36.780914306640628},"bottomLeft":{"x":3.877204656600952,"y":35.909996032714847},"bottomRight":{"x":1.6999046802520753,"y":35.909996032714847}},"velocity":{"x":0.0,"y":0.0},"size":0.8709198832511902,"speed":4.154879570007324,"mass":0.7585014700889587,"active":true},{"position":{"x":34.13535690307617,"y":30.488967895507814},"rectCorners":{"topLeft":{"x":32.92913818359375,"y":30.97145652770996},"topRight":{"x":35.341575622558597,"y":30.97145652770996},"bottomLeft":{"x":35.341575622558597,"y":30.006479263305665},"bottomRight":{"x":32.92913818359375,"y":30.006479263305665}},"velocity":{"x":0.0,"y":0.0},"size":0.9649763703346252,"speed":5.813979148864746,"mass":0.931179404258728,"active":true},{"position":{"x":8.436725616455079,"y":33.103759765625},"rectCorners":{"topLeft":{"x":7.1749773025512699,"y":33.60845947265625},"topRight":{"x":9.698473930358887,"y":33.60845947265625},"bottomLeft":{"x":9.698473930358887,"y":32.59906005859375},"bottomRight":{"x":7.1749773025512699,"y":32.59906005859375}},"velocity":{"x":0.0,"y":0.0},"size":1.0093984603881837,"speed":5.43798303604126,"mass":1.0188852548599244,"active":true},{"position":{"x":0.018898069858551027,"y":27.138145446777345},"rectCorners":{"topLeft":{"x":-1.2906899452209473,"y":27.6619815826416},"topRight":{"x":1.3284859657287598,"y":27.66197967529297},"bottomLeft":{"x":1.3284859657287598,"y":26.614309310913087},"bottomRight":{"x":-1.2906899452209473,"y":26.61431121826172}},"velocity":{"x":0.0,"y":0.0},"size":1.0476702451705933,"speed":4.2024407386779789,"mass":1.097612977027893,"active":true},{"position":{"x":28.230419158935548,"y":29.2669677734375},"rectCorners":{"topLeft":{"x":27.031618118286134,"y":29.746488571166993},"topRight":{"x":29.42922019958496,"y":29.746488571166993},"bottomLeft":{"x":29.42922019958496,"y":28.787446975708009},"bottomRight":{"x":27.031618118286134,"y":28.787446975708009}},"velocity":{"x":0.0,"y":0.0},"size":0.9590405225753784,"speed":5.003293991088867,"mass":0.9197587370872498,"active":true}],"walls":[{"topLeft":{"x":0.911354660987854,"y":22.673770904541017},"topRight":{"x":-0.0719839334487915,"y":23.068084716796876},"bottomLeft":{"x":2.861607551574707,"y":30.383872985839845},"bottomRight":{"x":3.8449459075927736,"y":29.989559173583986}},{"topLeft":{"x":48.3568229675293,"y":31.00674057006836},"topRight":{"x":49.52931594848633,"y":30.95561981201172},"bottomLeft":{"x":49.095088958740237,"y":20.996047973632814},"bottomRight":{"x":47.9225959777832,"y":21.047168731689454}},{"topLeft":{"x":37.999271392822269,"y":28.852949142456056},"topRight":{"x":37.844085693359378,"y":27.127193450927736},"bottomLeft":{"x":33.0825080871582,"y":27.55537223815918},"bottomRight":{"x":33.237693786621097,"y":29.2811279296875}},{"topLeft":{"x":39.48002624511719,"y":4.634716987609863},"topRight":{"x":39.917354583740237,"y":6.262379169464111},"bottomLeft":{"x":44.863990783691409,"y":4.933295249938965},"bottomRight":{"x":44.42666244506836,"y":3.305633068084717}},{"topLeft":{"x":42.4267578125,"y":38.37248229980469},"topRight":{"x":43.76646423339844,"y":38.137699127197269},"bottomLeft":{"x":42.694793701171878,"y":32.02253723144531},"bottomRight":{"x":41.35508728027344,"y":32.257320404052737}},{"topLeft":{"x":28.871000289916993,"y":9.19235610961914},"topRight":{"x":30.30930519104004,"y":9.346352577209473},"bottomLeft":{"x":30.995004653930665,"y":2.942023992538452},"bottomRight":{"x":29.556699752807618,"y":2.78802752494812}},{"topLeft":{"x":18.433305740356447,"y":-2.1436245441436769},"topRight":{"x":17.49428367614746,"y":-2.443850040435791},"bottomLeft":{"x":14.912160873413086,"y":5.632317543029785},"bottomRight":{"x":15.85118293762207,"y":5.9325432777404789}},{"topLeft":{"x":16.369842529296876,"y":-1.799057960510254},"topRight":{"x":16.575363159179689,"y":3.0857605934143068},"bottomLeft":{"x":19.009273529052736,"y":2.9833574295043947},"bottomRight":{"x":18.803752899169923,"y":-1.901461124420166}},{"topLeft":{"x":7.119476318359375,"y":20.621915817260743},"topRight":{"x":7.729763031005859,"y":21.59917449951172},"bottomLeft":{"x":14.008258819580079,"y":17.678327560424806},"bottomRight":{"x":13.397972106933594,"y":16.701068878173829}},{"topLeft":{"x":36.01359176635742,"y":27.68102264404297},"topRight":{"x":36.948585510253909,"y":27.01394271850586},"bottomLeft":{"x":32.575748443603519,"y":20.884883880615236},"bottomRight":{"x":31.64075469970703,"y":21.551963806152345}},{"topLeft":{"x":23.81734275817871,"y":32.89124298095703},"topRight":{"x":23.67662239074707,"y":35.73866271972656},"bottomLeft":{"x":27.499448776245118,"y":35.927589416503909},"bottomRight":{"x":27.640169143676759,"y":33.080169677734378}},{"topLeft":{"x":36.65388488769531,"y":40.560462951660159},"topRight":{"x":36.898563385009769,"y":39.48677062988281},"bottomLeft":{"x":28.98285675048828,"y":37.68291473388672},"bottomRight":{"x":28.738178253173829,"y":38.75660705566406}},{"topLeft":{"x":45.794715881347659,"y":15.891294479370118},"topRight":{"x":43.82923889160156,"y":15.054266929626465},"bottomLeft":{"x":42.0184326171875,"y":19.306333541870118},"bottomRight":{"x":43.983909606933597,"y":20.143360137939454}},{"topLeft":{"x":24.27729034423828,"y":29.030385971069337},"topRight":{"x":20.72364044189453,"y":28.748023986816408},"bottomLeft":{"x":20.500930786132814,"y":31.5509090423584},"bottomRight":{"x":24.054580688476564,"y":31.833271026611329}},{"topLeft":{"x":33.446739196777347,"y":21.60552406311035},"topRight":{"x":32.225975036621097,"y":21.752132415771486},"bottomLeft":{"x":33.135406494140628,"y":29.32474708557129},"bottomRight":{"x":34.356170654296878,"y":29.178138732910158}},{"topLeft":{"x":40.81627655029297,"y":16.256383895874025},"topRight":{"x":42.46690368652344,"y":15.101239204406739},"bottomLeft":{"x":39.682167053222659,"y":11.122026443481446},"bottomRight":{"x":38.03153991699219,"y":12.27717113494873}},{"topLeft":{"x":5.58711051940918,"y":30.872913360595704},"topRight":{"x":4.269734859466553,"y":30.790990829467775},"bottomLeft":{"x":3.8413543701171877,"y":37.67963790893555},"bottomRight":{"x":5.1587300300598148,"y":37.76156234741211}},{"topLeft":{"x":11.574536323547364,"y":6.659369468688965},"topRight":{"x":11.209813117980957,"y":7.584968566894531},"bottomLeft":{"x":19.675445556640626,"y":10.92076587677002},"bottomRight":{"x":20.04016876220703,"y":9.995166778564454}},{"topLeft":{"x":39.61371612548828,"y":-1.4476313591003419},"topRight":{"x":38.291099548339847,"y":-2.1588621139526369},"bottomLeft":{"x":35.25109100341797,"y":3.4943957328796388},"bottomRight":{"x":36.573707580566409,"y":4.205626487731934}},{"topLeft":{"x":12.205108642578125,"y":6.979550838470459},"topRight":{"x":11.892267227172852,"y":5.863853931427002},"bottomLeft":{"x":3.0339293479919435,"y":8.34773063659668},"bottomRight":{"x":3.346770763397217,"y":9.463428497314454}},{"topLeft":{"x":33.35993194580078,"y":-1.2491278648376465},"topRight":{"x":32.32726287841797,"y":-1.3852248191833497},"bottomLeft":{"x":31.114938735961915,"y":7.813554286956787},"bottomRight":{"x":32.147605895996097,"y":7.94965124130249}},{"topLeft":{"x":-0.04439115524291992,"y":21.315393447875978},"topRight":{"x":-0.8111686706542969,"y":22.67360496520996},"bottomLeft":{"x":3.7056541442871095,"y":25.223573684692384},"bottomRight":{"x":4.472431659698486,"y":23.8653621673584}},{"topLeft":{"x":-0.4699110984802246,"y":30.347429275512697},"topRight":{"x":-1.6460976600646973,"y":31.518482208251954},"bottomLeft":{"x":2.7063369750976564,"y":35.88999938964844},"bottomRight":{"x":3.882523536682129,"y":34.71894454956055}},{"topLeft":{"x":13.198797225952149,"y":14.111471176147461},"topRight":{"x":13.688961029052735,"y":12.426535606384278},"bottomLeft":{"x":8.261316299438477,"y":10.84758186340332},"bottomRight":{"x":7.771152496337891,"y":12.532517433166504}},{"topLeft":{"x":17.285659790039064,"y":15.965737342834473},"topRight":{"x":15.537479400634766,"y":16.091350555419923},"bottomLeft":{"x":15.952757835388184,"y":21.87087631225586},"bottomRight":{"x":17.700939178466798,"y":21.745262145996095}},{"topLeft":{"x":20.009521484375,"y":1.4205970764160157},"topRight":{"x":19.193164825439454,"y":0.8801589012145996},"bottomLeft":{"x":14.463019371032715,"y":8.0252685546875},"bottomRight":{"x":15.279376983642579,"y":8.565706253051758}},{"topLeft":{"x":48.63108825683594,"y":16.457304000854493},"topRight":{"x":47.68069076538086,"y":14.871625900268555},"bottomLeft":{"x":43.77264404296875,"y":17.213972091674806},"bottomRight":{"x":44.72304153442383,"y":18.799650192260743}},{"topLeft":{"x":-2.2525229454040529,"y":6.413692951202393},"topRight":{"x":-1.2374382019042969,"y":8.429162979125977},"bottomLeft":{"x":3.406266212463379,"y":6.090377330780029},"bottomRight":{"x":2.391181468963623,"y":4.074907302856445}},{"topLeft":{"x":6.799623489379883,"y":16.546873092651368},"topRight":{"x":5.619622230529785,"y":15.33869743347168},"bottomLeft":{"x":0.8667614459991455,"y":19.98072624206543},"bottomRight":{"x":2.046762704849243,"y":21.188901901245118}},{"topLeft":{"x":21.504674911499025,"y":7.116636753082275},"topRight":{"x":23.399202346801759,"y":6.788033485412598},"bottomLeft":{"x":22.6283016204834,"y":2.3434882164001467},"bottomRight":{"x":20.733774185180665,"y":2.672091484069824}}],"goalPosition":{"x":49.04840087890625,"y":8.865737915039063},"goalDiameter":2.5,"roomWidth":50.0,"roomHeight":40.0}'
fc = FlockControl(5)
print(fc.make_decisions(json.loads(response_raw)))