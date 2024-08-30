implicit none
        
        integer*4 nsec
        REAL*4    glat,glong
        integer*4 nxtrec
        byte in_sat,in_yr
        integer*2 iglat,iglong,in_dy,yr_int2
        integer*4 istop
        integer*2 ieflux(20),iiflux(20)
		logical*1 lcv
        integer :: hour, min, sec, readrec, n, i, s, errorcode
        
    	character(len=100) :: filename
    	
        !character(len=100) :: fish
    	
    	print *, 'Enter filename:'
    	read(*, *) filename
    	print *, 'Opening file', trim(filename)

             open(unit=60, file=filename, form='unformatted', access = 'DIRECT', status='old', recl=92, iostat=errorcode)

     	if (errorcode .gt. 0 ) then
			print *, 'fileopen error'
    		stop 
    	end if

    	lcv = .true.
    	do while(lcv)

   	    print *, 'Enter start record number, n:'
    	    read(*, *) readrec, n

    	    if ( readrec .gt. 0) then

    	  do i=readrec,readrec+n
          print *, 'this is right after the do statment'
   	        nxtrec = i
                read(unit=60,rec=nxtrec,iostat=istop) in_sat,in_yr, in_dy,iglat,iglong,ieflux,iiflux,nsec
          print *, 'this is before the if statement'

    	    if (istop .ne. 0) then 
    		print '(A10, I10)', 'Errorcode = ', errorcode
    	    else
    		hour = nsec/3600.0
    	    	min = (nsec - hour*3600.0)/60.0
    	    	sec = nsec - hour*3600.0 - min*60.0
                glat = iglat/10.
                glong = iglong/10.
            print *, 'this is before the write'
   	    	write(100, '(I2,A,I2,A,I2,A,I2,A,I4)') hour, ':', min, ':', sec, '19', yr_int2, in_dy,nsec,in_sat,glat,glong
    	    end if
    	  end do
    	  else 
    	    lcv = .false.
    	  end if    	
    	end do


  100  	format (x,i5,x,i2.2,a1,i2.2,a1,i2.2,x,a2,i2,x,i3,x,f10.0,x,i2,x,f6.1,x,f6.1,x,f6.1,x,f4.1)
  200  	format (x,10i4/x,10i4/x,10i4/x,10i4/x,a)

        pause 10
    	stop
    	end