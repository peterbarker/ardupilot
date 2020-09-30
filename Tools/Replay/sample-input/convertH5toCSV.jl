# Script permettant de convertir les données HDF5 en CSV pour faciliter l'utilisation dans AP

# Exemple :
# julia convertH5toCSV.jl ../input_data/2020_04_09_07_48_33_ADIS16465.h5 ../input_data/2020_04_09_07_48_33_ADIS16465.csv

# Ligne suivante : pour première utilisation
#import Pkg; Pkg.add("CSV"); Pkg.add("HDF5"); Pkg.add("DataFrames")

global function strip_ar!(string_ar)
	for i in 1:length(string_ar)
		string_ar[i] = strip(string_ar[i] )
	end
end

println(PROGRAM_FILE); for x in ARGS; println(x); end

input_file = ARGS[1]
output_file = ARGS[2]

using HDF5

fidi=h5open(input_file)
keys = names(fidi)


# récup des ID de tranches horaires
subset_hd_ar = []
for key in keys
	subset_hd = key[1:19]
	subset_tl = key[21:end]
	if subset_tl == "DATA"
		push!(subset_hd_ar,subset_hd)
	end	
end
sort!(subset_hd_ar)

# récup des données par tranche horaire
DATA = []
TIME = []
TIME_headers = []
DATA_headers = []
for subset_hd in subset_hd_ar
	println(subset_hd)
	push!(DATA , read(fidi[subset_hd * "_DATA"])' )
	push!(TIME , read(fidi[subset_hd * "_TIME"])' )
	global TIME_headers = read(fidi[subset_hd * "_TIME_headers"])
	global DATA_headers = read(fidi[subset_hd * "_DATA_headers"]) 
end

strip_ar!(DATA_headers)
strip_ar!(TIME_headers)
println(TIME_headers)

# création d'une grosse variable avec toutes les données et headers
using DataFrames
total_data_size = 0
total_column_size = size(DATA[1])[2]+size(TIME[1])[2]
for i in 1:length(DATA)
	global total_data_size += size(DATA[i])[1]
	#println(total_data_size)
end
#println(total_column_size)
out = zeros(total_data_size,total_column_size)
cur_id = 0
for i in 1:length(DATA)
	size_data = size(DATA[i])
	size_time = size(TIME[i])
	
	out[cur_id.+collect(1:size_time[1]),1:size_time[2]] = TIME[i]
	out[cur_id.+collect(1:size_time[1]),size_time[2]+1:end] = DATA[i]
	
	global cur_id += size_data[1]
end
#println(typeof(TIME_headers[:]))
#println([DATA_headers; TIME_headers])

out = DataFrame(out, Symbol.([TIME_headers; DATA_headers]))


# écriture dans CSV
using CSV

CSV.write(output_file, out; delim=';')




