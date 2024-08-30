!cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
!c
!c program to read dmsp/ssj4 files in APL CDROM archive format.
!c This software and data are in UNIX sysatem format.  If this is
!c to run on an INTEL based machine, the byte order must be swapped.
!c To accomplish this, uncomment the lines that call the rouitne
!c j4swap_endian.
!c
!ccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc
      implicit none

      character*80 filename
      integer*2 eflux(20),iflux(20),dy,iglat,iglon
      integer*4 ecnts(20),icnts(20)
      integer*4 t,c
      byte yr,isat
      integer*4    seconds
      integer*4 errorcode,lun,hour,min,sec,readrec,n,i
      logical*1 lcv
      
      print *,'enter filename'
      read 10, filename
 10   format(a)
      lun = 44

      open (unit=lun, file=filename, form='UNFORMATTED', status='old', access = 'DIRECT', recl=128, iostat = errorcode)

      if (errorcode .gt. 0 ) then
         print *, 'fileopen error'
         stop 

      end if

      open(1, file = 'dmsp_counts.txt', status='new', iostat = errorcode)
      
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
               read(unit=lun,rec=i,iostat=errorcode) isat, yr, dy, iglat, iglon, eflux, iflux, seconds
               
               if (errorcode .ne. 0) then 
                  print *, 'Error opening file '
                  print '(x,a,i10)', 'errorcode = ', errorcode
               else
!--------------------------------------------
! ****  Swap the bytes for INTEL type machines ****
                 call j4swap_endian (dy,iglat,iglon,eflux,iflux,seconds)
!
!---------------------------------------------
                  do t=1,20
                     c= eflux(t)
                     if(c .gt.32000) c=32000+(c-32000)*100
                     ecnts(t)  = c
                  end do
                  do t=1,20
                     c= iflux(t)
                     if(c .gt.32000) c=32000+(c-32000)*100
                     icnts(t)  = c
                  end do

                  hour = seconds/3600.0
                  min = (seconds - hour*3600.0)/60.0
                  sec = seconds - hour*3600.0 - min*60.0
                  
                  write(1,*) i,isat,seconds,hour,':',min,':',sec,'19',yr,dy,iglat/10.0,iglon/10.0
                  write(1,*) ecnts,icnts
                  
                  !print 100 ,i,isat,seconds,hour,':',min,':',sec,'19',yr,dy,iglat/10.0,iglon/10.0
                  !print 200, ecnts,icnts
                  
               end if
            end do
         else 
            lcv = .false.
         end if        
      end do


 100  format ('REC:',i5,' sat ',i2,1x,i5,1x,i2.2,a1,i2.2,a1,i2.2,1x,a2,i2,1x,i3,1x,f5.1,1x,f5.1)

200 format (1x,10i5/1x,10i5/1x,10i5/1x,10i5,a)
    
      stop
      end

!=================================================================

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

!-=======================================================

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