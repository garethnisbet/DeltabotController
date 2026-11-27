import pandas as pd

def read_the_data(scan: str, path: str): 
    data = pd.read_csv(f"{path}data.csv")
    return (data)

def relative_cx_cy(data, start_cx, start_cy): 
    data["cx_rel"] = data["cx"] - start_cx
    data["cy_rel"] = data["cy"] - start_cy

def px_to_mm(data, x_conversion, y_conversion): 
    data["cx_mm"] = -data["cx_rel"]*x_conversion #x-direction is right to left
    data["cy_mm"] = -data["cy_rel"]*y_conversion #cy-axis starts from 0 at the top 

def process_the_data(scan, path, x_conversion, y_conversion, start_cx, start_cy):
    data = read_the_data(scan, path)
    relative_cx_cy(data, start_cx, start_cy)   
    px_to_mm(data, x_conversion, y_conversion)
    return data

def combine_the_data(scans, path, x_conversion, y_conversion, start_cx, start_cy):
    """creates one dataframe of all the scans"""
    df_list = []
    for scan in scans:
        data = process_the_data(scan, path, x_conversion, y_conversion, start_cx, start_cy)
        # if scan[1]:
        #     data["cx_mm"] = -data["cx_mm"]
        df_list.append(data)
    combined_data = pd.concat(df_list,ignore_index=True)
    return combined_data

def list_the_data(scans, path, x_conversion, y_conversion, start_cx, start_cy):
    all_data = []
    for scan in scans:
        data = process_the_data(scan, path, x_conversion, y_conversion, start_cx, start_cy)
        all_data.append(data)
        print(all_data)
    return all_data

def dict_the_data(scans, path, x_conversion, y_conversion, start_cx, start_cy):
    """creates a dictionary of scan_name: data (df)"""
    all_data = {}
    for scan in scans:
        data = process_the_data(scan, path, x_conversion, y_conversion, start_cx, start_cy)
        all_data[f"{scan}"] = data
    return all_data

def series_to_supervised(data, n_in=1, n_out=1, dropnan=True):
	"""
	Frame a time series as a supervised learning dataset.
	Arguments:
		data: Sequence of observations as a list or NumPy array.
		n_in: Number of lag observations as input (X).
		n_out: Number of observations as output (y).
		dropnan: Boolean whether or not to drop rows with NaN values.
	Returns:
		Pandas DataFrame of series framed for supervised learning.
	"""
	n_vars = 1 if type(data) is list else data.shape[1]
	df = pd.DataFrame(data)
	cols, names = list(), list()
	# input sequence (t-n, ... t-1)
	for i in range(n_in, 0, -1):
		cols.append(df.shift(i))
		names += [('var%d(t-%d)' % (j+1, i)) for j in range(n_vars)]
	# forecast sequence (t, t+1, ... t+n)
	for i in range(0, n_out):
		cols.append(df.shift(-i))
		if i == 0:
			names += [('var%d(t)' % (j+1)) for j in range(n_vars)]
		else:
			names += [('var%d(t+%d)' % (j+1, i)) for j in range(n_vars)]
	# put it all together
	agg = pd.concat(cols, axis=1)
	agg.columns = names
	# drop rows with NaN values
	if dropnan:
		agg.dropna(inplace=True)
	return agg
