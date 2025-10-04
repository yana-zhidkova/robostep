from statistics import mode
tuple_list = [(1,2,3), (4,5,6), (4,6,8), (1,2,3), (1,2,5), (4,5,6)]
str_tuple_list = list(map(str, tuple_list))
print(str_tuple_list)
print(mode(str_tuple_list))
 