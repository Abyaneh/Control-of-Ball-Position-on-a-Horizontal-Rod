%Ac Project 2
%%Khayami-Padash-Abyaneh
%%%10-1-10 data
%The first method of importing data in this section and the second method is mentioned in 10-0-10.
%Home ==> Import Data ==> Select Excel ==> Choose the desired columns
%The name of the Excel file here was “data,” and under this title, a variable was created.
clc;

time=0.01*(1:2694);
%For the position, column 1 of the Excel file was used
position=reshape(data,1,2694);   %to shape the time and position matrix
plot(time,position);
s=stepinfo(time,position); %to find characteristics such as rise time, …
%Now, one can use App ==> Curve Fitting to obtain a better form.
%You can delete the data from the workspace and then re-import the second column of the Excel file,
%Select the second column of the Excel file for import, which corresponds to the degree.
%which corresponds to the motor rotation degree *100, where the *100 factor is used
%to create a hypothetical distinction.
deg=reshape(data,1,2695);
plot(time,deg);
%======================================================10-1-10