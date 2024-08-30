;Patrick Newell, September 2005.  IDL program to take a day buffer and
;create a single pass.  Each call produces the next
;pass, defined as when the satellite reverses from ascending to
;descending, or vice versa
;Modified August 2008 to include nightside, and not to reverse time on
;descending latitude passes

pro fill_pass_buff,day_buff,pass_buff,last_end,next_start,next_end

;day_buff(last_end) = end of last pass (supplied by call)
;day_buff(next_start) = first record of new pass (returned)
;next_end also returned
;pass_buff is a buffer with just the next pass (also returned)


end_day = n_elements(day_buff)
if( (end_day - last_end) lt 200L )then begin
   next_start = -2
   return
endif

ic = long(last_end) + 10L
mlatc = abs(day_buff(ic).mlat)
while ( (50. ge mlatc) )do begin
 ic = ic + 1L
 if( (end_day - ic) lt 200 )then begin
   next_start = -2
   return
 endif
 mlatc = abs(day_buff(ic).mlat)
endwhile

next_start = ic
mlat0 = mlatc
;ascending or descending?
while ( abs(mlat0-mlatc) lt 0.3 )do begin
  ic = ic + 1L
  mlatc = abs(day_buff(ic).mlat)
endwhile

ascend = 1
mlatp = mlatc
if( mlatc lt mlat0 )then begin
   ascend = -2
   mlatp = mlat0
endif


done = -2
if( ascend )then begin
 while( not done )do begin
 ic = ic + 1L
 if( ic ge end_day )then begin
  next_start = -2
  return
endif
mlatc = abs(day_buff(ic).mlat)
; mltc = day_buff(ic).mlt
 if( mlatc lt mlatp  )then begin
   done = 1
 endif
 if( (not done) and (mlatc gt mlatp) )then begin
   mlatp = mlatc
 endif
 endwhile
endif else begin
 while( not done )do begin
 ic = ic + 1L
if( ic ge end_day )then begin
   next_start = -2
   return
endif
 mlatc = abs(day_buff(ic).mlat)
; mltc = day_buff(ic).mlt
 if( mlatc lt 50. )then begin
  done = 1
endif
endwhile
endelse


next_end = ic

pass_buff = day_buff(next_start:next_end)


return
end


 





