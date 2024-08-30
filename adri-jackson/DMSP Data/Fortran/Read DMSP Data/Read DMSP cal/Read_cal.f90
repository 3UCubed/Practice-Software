!ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
!
! program to read dmsp/ssj4 files in APL CDROM archive format
! and output the calibrated data.  If thisis to run on an INTEL
! bases machine, the bytes from the file must be swapped.  This
! can be accomplished by uncommenting the lines that call the
! routine j4swap_endian.
!
!cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc

      implicit none

      character*80 filename
      real*4    deflux(20),diflux(20), glon,glat
      real*4    jne, je, jni, ji
      integer*2 isat,yr,dy
      integer*4 t,c

      integer*4 seconds
      integer*4 errorcode,lun,hour,min,sec,readrec,n,i
      logical*1 lcv
      
      print *,'enter filename'
      read 10, filename
 10   format(a)
      lun = 44

      open (unit=lun,file=filename,form='unformatted',status='old',access = 'DIRECT',recl=92,iostat = errorcode)


      if (errorcode .gt. 0 ) then
         print *, 'fileopen error'
         stop 
      end if

      
      lcv = .true.
      do while(lcv)

         print *, 'enter start record number, n'
         read *, readrec,n

         if ( readrec .gt. 0) then

            do i=readrec,readrec+n-1

               call j4read_cal_rec (lun,i,isat,yr,dy,glat,glon,deflux,diflux,jne, je, jni, ji,seconds, errorcode)
               if (errorcode .ne. 0) then 
                  print *, 'Error reading file '
                  print '(x,a,i10)', 'errorcode = ', errorcode
               else

                  hour = seconds/3600.0
                  min = (seconds - hour*3600.0)/60.0
                  sec = seconds - hour*3600.0 - min*60.0
                  print 100 ,i,isat,seconds,hour,':',min,':',sec,'19',yr,dy,glat,glon

                  print 200, deflux, diflux

               end if
            end do
         else 
            lcv = .false.
         end if        
      end do


 100  format ('REC:',i5,' sat ',i2,1x,i5,1x,i2.2,a1,i2.2,a1,i2.2,1x,a2,i2,1x,i3,1x,f5.1,1x,f5.1)

 200  format (1x,10E12.5/1x,10E12.5/1x,10E12.5/1x,10E12.5,a)

      stop
      end

!C=================================================================

      subroutine j4swap_endian (dy,iglat,iglon,eflux,iflux,seconds)

      implicit none

      integer*2 i, itemp
      integer*2 eflux(20),iflux(20),dy,iglat,iglon
      integer*4 seconds, secs4
      integer*2 secs2(2)
      
      equivalence (secs2,secs4)

      secs4 = seconds

      do i = 1, 20
         call swap_bytes (eflux(i))
         call swap_bytes (iflux(i))
      enddo

      call swap_bytes (dy)
      call swap_bytes (iglat)
      call swap_bytes (iglon)
      call swap_bytes (secs2(1))
      call swap_bytes (secs2(2))

      itemp = secs2(1)
      secs2(1) = secs2(2)
      secs2(2) = itemp

      seconds = secs4
      
      return
      end

!C -=======================================================

      subroutine swap_bytes (inp)

      implicit none
      
      integer*2 inp, inp2
      byte      binp(2), temp

      equivalence (inp2,binp)

      inp2 = inp

      temp = binp(1)
      binp(1) = binp(2)
      binp(2) = temp

      inp = inp2
      return
      end

!C ============================================================

      subroutine j4read_cal_rec (lun,nrec,isat,yr,dy,glat,glon,deflux,diflux,jne, je, jni, ji,nsec, istop)

      implicit none

!c**************************************************************
!c  This defines the common block to hold a satellite
!c   calibration factors.
!c**************************************************************
      REAL*4  	EENG(20), GFe(20), GFi(20)
      REAL*4    IENG(20),ewid(20),iwid(20)
      REAL*4 	k1e(20), k2e(20), k1i(20),k2i(20)
      INTEGER*4	isat_fact, iyear_fact, isat_req

      COMMON /sat_facts/ isat_req, isat_fact, iyear_fact, EENG,IENG,EWID,IWID,GFe,GFi,k1e,k2e,k1i,k2i
!c**************************************************************

      integer*4 lun, nrec
      integer*2 isat,yr,dy
      real*4    glat, glon
      real*4    deflux(20),diflux(20)
      real*4    jne, je, jni, ji
      integer*4 nsec, istop

      byte      in_sat,in_yr
      integer*2 iglat,iglong,in_dy
      integer*2 ieflux(20),iiflux(20)

      integer*4 yearsec,ierr4,sat_id,year
      real*4    temp
      real*4    eflux(20),iflux(20)
      integer*4 i
      real*4    tmpiflux(20)

      read(unit=lun,rec=nrec,iostat=istop)in_sat,in_yr,in_dy,iglat,iglong,ieflux,iiflux,nsec

!c ---------------------------------------------
!c The next routine is called to swap the bytes
!c on an INTEL based machine.
!c ---------------------------------------------
!c      call j4swap_endian (in_dy,iglat,iglong,ieflux,iiflux,
!c     %                                nsec)

      isat = in_sat
      sat_id = in_sat
      year = in_yr
      yr = in_yr
      dy = in_dy

      if ((isat .NE. isat_req) .OR. (year .NE. iyear_fact)) then
         CALL get_ssj_facts ( sat_id, year)
      endif
      
      if (isat_req .NE. isat_fact) then
         print *,'GETDKS: No conversion factors for satellite', isat
         stop
      endif

      if((yr.eq.83).and.(dy.lt.100)) yr=84
      
      do i=1,20
         temp = tmpiflux(i)
         if(temp.gt.32000.) temp=32000.+(temp-32000.)*100.
         iflux(i) = temp

         temp = ieflux(i)
         if(temp.gt.32000.) temp = 32000.+(temp-32000.)*100.
         eflux(i) = temp
      end do


      je = 0.
      jne = 0.
      ji = 0.
      jni = 0.
      do i = 1,9
         deflux(i) = eflux(i)*eeng(i)/gfe(i)/0.098
         diflux(i) = iflux(i)*ieng(i)/gfi(i)/0.098
         jne = jne + k1e(i)*eflux(i)
         je = je + k2e(i) * 1000.0 * eflux(i)
         jni = jni + k1i(i)*iflux(i)
         ji = ji + k2i(i) * 1000.0 * iflux(i)
      end do

         deflux(10) = eflux(10)*eeng(10)/gfe(10)/0.098
         diflux(10) = iflux(10)*ieng(10)/gfi(10)/0.098

      do i = 11,20
         deflux(i) = eflux(i)*eeng(i)/gfe(i)/0.098
         diflux(i) = iflux(i)*ieng(i)/gfi(i)/0.098
         jne = jne + k1e(i)*eflux(i)
         je = je + k2e(i) * 1000.0 * eflux(i)
         jni = jni + k1i(i)*iflux(i)
         ji = ji + k2i(i) * 1000.0 * iflux(i)
      end do

      glat = iglat/10.
      glon = iglong/10.

      yearsec = 86400*(dy-1) + nsec

      return
      end

!C ===========================================================
!c******************************************************************
!c  ROUTINE:
!c     get_ssj_facts
!c
!c  PURPOSE:
!c     To read the calibration factors for a given satellite and
!c     store them into a common block for later use
!c
!c  CALLING SEQUENCE:
!c     CALL get_ssj_facts (isat, iyear)
!c
!c  INPUTS:
!c     isat  - INTEGER*4 - The satellite id number (starting with 6)
!c     iyear - INTEGER*4 - The year of the data
!c
!c  OUTPUTS:
!c     A comon block containing the calibration factors for the
!c     desired satellite
!c
!c  HISTORY:
!c     2/2005  - J. Skura  Created using new calibration factors
!c                         and corrections from AFRL
!c******************************************************************
      SUBROUTINE get_ssj_facts(isat, iyear)
      implicit none

!c**************************************************************
!c  This defines the common block to hold a satellite
!c   calibration factors.
!c**************************************************************
      REAL*4  	EENG(20), GFe(20), GFi(20)
      REAL*4    IENG(20),ewid(20),iwid(20)
      REAL*4 	k1e(20), k2e(20), k1i(20),k2i(20)
      INTEGER*4	isat_fact, iyear_fact, isat_req

      COMMON /sat_facts/ isat_req, isat_fact, iyear_fact,EENG,IENG,EWID,IWID,GFe,GFi,k1e,k2e,k1i,k2i
!c**************************************************************

      character*80  cal_dir
      character*80  cal_default
      character*80  cal_envvar
      logical*1     cal_chars(80)

      INTEGER*4     isat,icnt,jsat, isatnum, kk
      INTEGER*4     iyear, iyr, file_num, iyr1, iyr2

      REAL*4 EHi,ELo, IHi, ILo

      character*132 filename(2)

      INTEGER*2     rec_read
      INTEGER*4     errorcode, i, j

      EQUIVALENCE (cal_dir,cal_chars)

      DATA cal_envvar /'SAT_FACTORS_DIR'/
      DATA cal_default /'./'/
      
      CALL getenv (cal_envvar,cal_dir)

      if (ICHAR(cal_chars(1)) .EQ. 32) then
         WRITE(cal_dir,"(A)")cal_default
      endif

      write(filename(1),"(A)")cal_dir
      WRITE(filename(2),"(A)")cal_dir

      j=0
      do i=1,80
         if ((j .eq. 0).AND.(ICHAR(cal_chars(i)) .eq. 32)) j=i
      END DO

      write (filename(1)(j:j+16),10) 
 10   FORMAT(16H/ssj_factors.dat)
      write (filename(2)(j:j+26),20) 
 20   FORMAT(26H/j4_yearly_corrections.dat)

      isat_req = isat
      iyr = iyear
      iyear_fact = iyear
      IF (iyr .LT. 1900) iyr = iyr + 1900
      IF (iyr .LT. 1980) iyr = iyr + 100

      file_num = 1

      OPEN (unit=23,file=filename(file_num),form='formatted',status='old',action = 'read',iostat = errorcode)

      IF (errorcode .GT. 0) THEN
         print *,' Error opening file: '//filename
         DO icnt=1,20
            Eeng(icnt) = 0.0
            GFe(icnt) = 0.0
            GFi(icnt) = 0.0
         END DO
      ELSE
         rec_read = isat - 5

!c ****** This section will cause processing to stop for satellites
!c ****** before F6
         IF (rec_read .LE. 0) THEN
            print *, 'Calibration factors not available for sat =',isat
            STOP
         ENDIF

         icnt = 1
         errorcode = 0
         jsat = -5
         do while ((jsat .NE. isat) .AND. (errorcode .EQ. 0))
            READ (23,*,iostat=errorcode)jsat
            PRINT * ,'Read satellite ',jsat
            READ (23,*,iostat=errorcode)(Eeng(kk), kk=1,10)
            READ (23,*,iostat=errorcode)(Eeng(kk), kk=11,20)

            READ (23,*,iostat=errorcode)(Ieng(kk), kk = 1,10)
            READ (23,*,iostat=errorcode)(Ieng(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(ewid(kk), kk = 1,10)
            READ (23,*,iostat=errorcode)(ewid(kk), kk = 11,20)

            READ (23,*,iostat=errorcode)(iwid(kk), kk= 1,10)
            READ (23,*,iostat=errorcode)(iwid(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(GFe(kk), kk = 1,10)
            READ (23,*,iostat=errorcode)(GFe(kk), kk = 11,20)

            READ (23,*,iostat=errorcode)(GFi(kk), kk= 1,10)
            READ (23,*,iostat=errorcode)(GFi(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(k1e(kk), kk= 1,10)
            READ (23,*,iostat=errorcode)(k1e(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(k2e(kk), kk= 1,10)
            READ (23,*,iostat=errorcode)(k2e(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(k1i(kk), kk= 1,10)
            READ (23,*,iostat=errorcode)(k1i(kk), kk= 11,20)

            READ (23,*,iostat=errorcode)(k2i(kk), kk = 1,10)
            READ (23,*,iostat=errorcode)(k2i(kk), kk = 11,20)

         enddo

 33      FORMAT(20E13.6)

!c ****** This next section will stop processing if satellite data
!c ****** is not available.  To use last satelite data, comment out
!c ****** this section 
         IF (errorcode .NE. 0) THEN
            print *, 'Calibration factors not available for sat =',isat
            STOP
         ENDIF
!c ****** end of section for halting on error

         isat_fact = jsat

         CLOSE (UNIT=23)

!C ********* get correction factors next

         OPEN (UNIT= 25, file = filename(2),form='formatted',status='old',action = 'read', iostat = errorcode)
         IF (errorcode .GT. 0) THEN
            print *, 'Error Opening file : '//filename(2)
         ELSE
!C *** Skip First line in file
            JSAT = -5
            READ(25,*)
            DO WHILE (jsat .NE. isat)
!C ** GET SAT NUMBER AND YEARS COVERED
               READ(25,*) jsat, iyr1, iyr2
               if (jsat .EQ. isat) THEN
                  if (iyr .LT. iyr1) iyr2 = iyr1
                  if ((iyr .LT. iyr2) .AND. (iyr .GE. iyr1)) iyr2 = iyr
               ENDIF
               DO kk = iyr1, iyr2
                  READ(25,*) EHi, ELo, IHi, ILo
               ENDDO
            ENDDO
            CLOSE (UNIT=25)
         ENDIF
         PRINT *, 'FACTORS =',EHi, ELo, IHi, ILo

!C **** APPLY THE CORRECTIONS ****
         DO icnt = 1, 10 
            GFe(icnt) = GFe(icnt) * ELo
            GFe(icnt+10) = GFe(icnt+10) * EHi
            GFi(icnt) = GFi(icnt) * ILo
            GFi(icnt+10) = GFi(icnt+10) * IHi
            k1e(icnt) = k1e(icnt) / ELo
            k1e(icnt+10) = k1e(icnt+10) / EHi
            k2e(icnt) = k2e(icnt) / ELo
            k2e(icnt+10) = k2e(icnt+10) / EHi
            k1i(icnt) = k1i(icnt) / ILo
            k1i(icnt+10) = k1i(icnt+10) / IHi
            k2i(icnt) = k2i(icnt) / ILo
            k2i(icnt+10) = k2i(icnt+10) / IHi
         ENDDO
            


      END IF

      RETURN
      END

