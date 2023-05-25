syms a b c d

H4_positive =[a,-b,-c,-d;b,a,-d,c;c,d,a,-b;d,-c,b,a]


H4_negative = [a,-b,-c,-d;b,a,d,-c;c,-d,a,b;d,c,-b,a]

H8_positive = [H4_positive,zeros(4,4);H4_positive,H4_positive]

H8_negative = [H4_negative,zeros(4,4);H4_negative,H4_negative]