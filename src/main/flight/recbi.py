
myList = list(range(200))

def recbi(myList, val):
    N = len(myList)
    ind = [0, N-1]
    i = 0
    if (val >= myList[-1]):
        return [N-1,N-1]
    elif (val < myList[0]):
        return [-1, 0]

    while ((ind[1] - ind[0]) > 1) and (i < N):
        midpoint = int( (ind[0] + ind[1]) / 2 )
        if myList[midpoint] <= val:
            ind[0] = midpoint
        else:
            ind[1] = midpoint
        i = i + 1

    return ind, i

recbi(myList, 101)
