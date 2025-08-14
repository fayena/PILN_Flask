 select printf('$%.2f',round(sum(pid_output)/2/60/60*12*.10,2)) from firing where run_id>44;


--kiln rated kwh = 12
--$/kwh = $0.11
-- 1/(100*60*60)=1/36000
select printf('$%.2f',round(sum(f.pid_output*s.int_sec)*12/360000*.11,2))
  from firing f,
       segments s
  where f.run_id>44
    and s.run_id=f.run_id
    and s.segment=f.segment;

select printf('$%.2f',round(sum(f.pid_output*s.int_sec)*kwh/100/60/60*.$/kwh,2)) 

