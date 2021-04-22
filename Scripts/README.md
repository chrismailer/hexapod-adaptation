# Useful Terminal Snippets

## PBS Operations
The .pbs files contain all of the info for running a job on the CHPC. Have a look at one of them to get an idea of the structure. The name field is intentionally left empty and needs to be specified when starting a job.
```
qsub -N <jobName> generate.pbs
```
```
qdel <jobID>
```
```
qstat -f <jobID>
```
```
qstat -u <Username>
```

## Setting up map size experiment folders
```
for i in 1 2 3 4 5 6 7 8 9 10
do
cp -r map_n map_$i
done
```
```
for i in 1 2 3 4 5 6 7 8 9 10
do
cd ~/lustre/80k/map_$i
qsub -N 80k_$i generate.pbs
done
```

# Useful Links
CHPC quick start - http://wiki.chpc.ac.za/quick:start