pro get_sat_facts,sat,facts,year, path = path
  
  IF (not keyword_set(path)) THEN BEGIN
    path = '/project/dmsp/ssj4/cal_factors/'
  ENDIF
    
  factors_fname = path + 'ssj_factors.dat'
  corrections_fname = path + 'j4_yearly_corrections.dat'
  
;
;   only the last two digits of year are kept; 
;   here a Y2K fix is attempted
;
  if(year lt 80) then begin
    year = year + 2000
  endif else begin
    if(year lt 1900) then begin
      year = year + 1900
    endif
  endelse
  
  openr,lun,factors_fname,/get_lun
  
  if(lun eq -1) then begin
    print,'GET_SAT_FACTS: error opening conversion factors file'
    return
  endif
  
  if(sat lt 6) then begin
    print,'GET_FACTS: bad sat number'
    return
  endif
  
  facts= { eeng:fltarr(20),ieng:fltarr(20), $
           ewid:fltarr(20),iwid:fltarr(20), $
           gfe:fltarr(20), gfi:fltarr(20),  $
           k1e:fltarr(20), k2e:fltarr(20),  $
           k1i:fltarr(20), k2i:fltarr(20) }
  
  eeng1 = fltarr(10)
  eeng2 = fltarr(10)
  ieng1 = fltarr(10)
  ieng2 = fltarr(10)
  ewid1 = fltarr(10)
  ewid2 = fltarr(10)
  iwid1 = fltarr(10)
  iwid2 = fltarr(10)
  gfe1 = fltarr(10)
  gfe2 = fltarr(10)
  gfi1 = fltarr(10)
  gfi2 = fltarr(10)
  k1e1 = fltarr(10)
  k1e2 = fltarr(10)
  k2e1 = fltarr(10)
  k2e2 = fltarr(10)
  k1i1 = fltarr(10)
  k1i2 = fltarr(10)
  k2i1 = fltarr(10)
  k2i2 = fltarr(10)
  
  sat_num = -5
  
  WHILE ((sat_num NE sat) AND NOT EOF(lun)) DO BEGIN
    line = ""
    readf,lun,line
    reads,line,sat_num
    readf,lun,eeng1
    readf,lun,eeng2
    readf,lun,ieng1
    readf,lun,ieng2
    readf,lun,ewid1
    readf,lun,ewid2
    readf,lun,iwid1
    readf,lun,iwid2
    readf,lun,gfe1
    readf,lun,gfe2
    readf,lun,gfi1
    readf,lun,gfi2
    readf,lun,k1e1
    readf,lun,k1e2
    readf,lun,k2e1
    readf,lun,k2e2
    readf,lun,k1i1
    readf,lun,k1i2
    readf,lun,k2i1
    readf,lun,k2i2
  ENDWHILE
  
  line = ""
; create correction factor array to contain factors for
;   EHI, ELO, IHI, ILO
  factors = fltarr(4)
  
  info = intarr(3)
  openr,clun,corrections_fname,/get_lun
; skip first line
  readf,clun,line
  cnum = -5
  WHILE ((sat_num NE cnum) AND NOT EOF(clun)) DO BEGIN
; read in the satellite number and years
    readf,clun,info
    cnum = info[0]
    IF (cnum EQ sat) THEN BEGIN
      if (year LT info[1]) THEN INFO[2] = INFO[1]
      if ((year LT info[2]) AND (year GE info[1])) THEN info[2] = year
    ENDIF
    FOR i = info[1], info[2] DO BEGIN
      readf,clun,factors
    ENDFOR
  ENDWHILE

  close,clun
  Free_lun,clun
  FOR i = 0,3 DO $
    IF ((factors[i] GT  1.) OR (factors[i] LE 0.001)) THEN factors[i] = 1.
  
  FOR i=0,9 DO BEGIN
    facts.eeng[i] = eeng1[i]
    facts.eeng[10+i] = eeng2[i]
    facts.ieng[i] = ieng1[i]
    facts.ieng[10+i] = ieng2[i]
    facts.ewid[i] = ewid1[i]
    facts.ewid[10+i] = ewid2[i]
    facts.iwid[i] = iwid1[i]
    facts.iwid[10+i] = iwid2[i]
    facts.gfe[i] = gfe1[i] * factors[1]
    facts.gfe[10+i] = gfe2[i] * factors[0]
    facts.gfi[i] = gfi1[i] * factors[3]
    facts.gfi[10+i] = gfi2[i] * factors[2]
    facts.k1e[i] = k1e1[i] / factors[1]
    facts.k1e[10+i] = k1e2[i] / factors[0]
    facts.k2e[i] = k2e1[i] / factors[1]
    facts.k2e[10+i] = k2e2[i] / factors[0]
    facts.k1i[i] = k1i1[i] / factors[3]
    facts.k1i[10+i] = k1i2[i] / factors[2]
    facts.k2i[i] = k2i1[i] / factors[3]
    facts.k2i[10+i] = k2i2[i] * factors[2]
  ENDFOR
  
  close,lun
  free_lun,lun
  return
end
