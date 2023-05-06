const toHex = (value: number | uint8 | uint16, len: 2 | 4): string => {
  if (len === 4) return ('0000' + value.toString(16)).slice(-4);
  if (len === 2) return ('00' + value.toString(16)).slice(-2);
};
