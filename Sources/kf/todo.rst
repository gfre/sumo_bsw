======================
Kalman Filter TODO's
======================
1. Make KF independent of DRIVE task, so that different KFs can be calculated from different modules at different times (smplTimeMS in KF_Cfg_t unused now)
2. since :math:`DP` is a diagonal matrix and :math:`UP` a unit upper triangular matrix, they can be stored in one matrix (or even a vector) with the off-diagonal elements stored in :math:`DP`
   e.g.: 
	
   .. math::
	   DP&=\begin{bmatrix}
			12 &  0 & 0\\
			0  & 15 & 0\\
			0  &  0 & 17
		  \end{bmatrix},\\
	   UP&=\begin{bmatrix}
			1  &  2 & 3\\
			0  &  1 & 4\\
			0  &  0 & 1
		  \end{bmatrix}
			
   can be stored as 
	
   .. math::
	   UDP_{comb} = \begin{bmatrix}12 &  2 & 3\\ 0  & 15 & 4\\0  &  0 & 17\end{bmatrix}
			
3. Make general API-functions for state estimates (at the moment, KF returns the second state hard-coded with KF_Read_i16EstdVal)
4. Add feature that :math:`R` and :math:`Q`, respectively, can contain coloured noise. In order to achieve this, :math:`R` and :math:`Q` must be decorrelated using UD-decomposition such that
   :math:`R = U_{R} D_{R} U_{R}^T` and :math:`Q = U_Q D_Q U_Q^T`. Only then the measurements can be calculated sequentially. In order to do this, the measurement vector must 
   be decorrelated. (see Kalman Filtering - Theory And Practice using MATLAB by Grewal Ed. 4 p.303).
   Therefore, the equations
	
   .. math::
	  \acute{\boldsymbol{y}} &= U_R^{-1}\boldsymbol{y},\\
	  \acute{H} &= U_R^{-1}H
		
   must be solved before the Bierman measurement update can be calculated. This transformation has the effect that :math:`R_{dash} = D_R`
   is a diagonal matrix and thus, the measurements in the Bierman update can be calculated independently.
   For the Thornton temporal update, the parameter "mGUQ\_" already implies that :math:`G` must be multiplied to :math:`U_Q` if it is given (and :math:`Q` is non-diagonal). This has 
   to be adapted, too. At this moment, :math:`G` must be initiated as the identity matrix in the config because it is handled to the function in KF_Predict_P().
5. Reduce number of temporal variables 

	