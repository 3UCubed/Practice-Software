;This program can serve as a simple example.  It just reads in the
;binary data file and outputs some parameters in ascii (into
;'converted.txt')
;Patrick Newell

pro cnv2asc
in_file = '2010jan01.f17'
print,in_file
out_file = 'converted.txt'
get_lun, lun
openw,lun,out_file
print,'Start second of day: (nnnnn):'
sod_start = 0L
sod_stop = 0L
read,sod_start
print,'Stop sod: '
read,sod_stop

printf,lun,in_file
printf,lun,sod_start,sod_stop

s1 = read_ssj_file(in_file, dat_raw)
s2 = conv_struc(dat_raw, day_buff)

last_end = 0L
fill_pass_buff,day_buff,pass_buff,last_end,next_start,next_end

while( next_start ge 0 )do begin
n = n_elements(pass_buff)
;print,"next start: ",next_start, day_buff(next_start).sod,'n=',n
sec = pass_buff(0).sod
;s2h,sec,hh,mm,ss
;print,sec,hh,mm,ss,pass_buff(0).mlat,pass_buff(0).mlt
for i=0,n-1 do begin
 sod = pass_buff(i).sod
 mlat = pass_buff(i).mlat
 mlt = pass_buff(i).mlt
 glat = pass_buff(i).glat
 glong = pass_buff(i).glon
 je = pass_buff(i).jee
 ji = pass_buff(i).jei
 eave = pass_buff(i).eav
iave = pass_buff(i).iav
if( (sod gt sod_start) and (sod lt sod_stop) )then begin
printf,lun,sod,mlat,mlt,glat,glong,je,ji,eave,iave
printf,lun,'   '
endif
endfor


last_end = next_end
fill_pass_buff,day_buff,pass_buff,last_end,next_start,next_end
endwhile

close, lun
free_lun, lun
print, "DONE"

end

