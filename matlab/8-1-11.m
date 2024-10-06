%Khayami-Padash-Abyaneh
%Second method of importing data:
%Home ==> Import Data ==> Select both columns and create a Numeric Matrix
%which will result in a variable ‘X*2’ being created under the title ‘data’. Then, for any operation,
%you must use the desired column or row.
data=reshape(data,2,1953);
time=0.01*(1:1953);
position=data(1,:);%Selecting all columns of the first row
plot(time,position);
s=stepinfo(time,position);%to find characteristics such as rise time, …
deg=data(2,:);
plot(time,deg);
%======================================8-1-11