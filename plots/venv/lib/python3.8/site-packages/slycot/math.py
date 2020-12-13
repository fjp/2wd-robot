#
#       math.py
#
#       Copyright 2010 Enrico Avventi <avventi@Lonewolf>
#
#       This program is free software; you can redistribute it and/or modify
#       it under the terms of the GNU General Public License version 2 as
#       published by the Free Software Foundation.
#
#       This program is distributed in the hope that it will be useful,
#       but WITHOUT ANY WARRANTY; without even the implied warranty of
#       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#       GNU General Public License for more details.
#
#       You should have received a copy of the GNU General Public License
#       along with this program; if not, write to the Free Software
#       Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#       MA 02110-1301, USA.

from . import _wrapper
from .exceptions import raise_if_slycot_error

import numpy as np


def mb03rd(n, A, X=None, jobx='U', sort='N', pmax=1.0, tol=0.0):
    """Ar, Xr, blsize, W = mb03rd(n, A, [X, jobx, sort, pmax, tol])

    To reduce a matrix `A` in real Schur form to a block-diagonal form
    using well-conditioned non-orthogonal similarity transformations.
    The condition numbers of the transformations used for reduction
    are roughly bounded by `pmax`*`pmax`, where `pmax` is a given value.
    The transformations are optionally postmultiplied in a given
    matrix `X`. The real Schur form is optionally ordered, so that
    clustered eigenvalues are grouped in the same block.

    Parameters
    ----------
        n : int
            The order of the matrices `A` and `X`.  `n` >= 0.
        A : (n, n) array_like
            The matrix `A` to be block-diagonalized, in real Schur form.
        X : (n, n) array_like, optional
            A given matrix `X`, for accumulation of transformations (only if
            `jobx`='U')
        jobx : {'N', 'U'}, optional
            Specifies whether or not the transformations are
            accumulated, as follows:

            := 'N': The transformations are not accumulated
            := 'U': The transformations are accumulated in `Xr` (default)

        sort : {'N', 'S', 'C', 'B'}, optional
            Specifies whether or not the diagonal blocks of the real
            Schur form are reordered, as follows:

            := 'N':  The diagonal blocks are not reordered (default);
            := 'S':  The diagonal blocks are reordered before each
                     step of reduction, so that clustered eigenvalues
                     appear in the same block;
            := 'C':  The diagonal blocks are not reordered, but the
                     "closest-neighbour" strategy is used instead of
                     the standard "closest to the mean" strategy
                     (see Notes_);
            := 'B':  The diagonal blocks are reordered before each
                     step of reduction, and the "closest-neighbour"
                     strategy is used (see Notes_).

        pmax : float, optional
            An upper bound for the infinity norm of elementary
            submatrices of the individual transformations used for
            reduction (see Notes_).  `pmax` >= 1.0
        tol : float, optional
            The tolerance to be used in the ordering of the diagonal
            blocks of the real Schur form matrix.
            If the user sets `tol` > 0, then the given value of `tol` is
            used as an absolute tolerance: a block `i` and a temporarily
            fixed block 1 (the first block of the current trailing
            submatrix to be reduced) are considered to belong to the
            same cluster if their eigenvalues satisfy

            .. math:: | \\lambda_1 - \\lambda_i | <= tol.

            If the user sets `tol` < 0, then the given value of tol is
            used as a relative tolerance: a block i and a temporarily
            fixed block 1 are considered to belong to the same cluster
            if their eigenvalues satisfy, for ``j = 1, ..., n``

            .. math:: | \\lambda_1 - \\lambda_i | <= | tol | * \\max | \\lambda_j |.

            If the user sets `tol` = 0, then an implicitly computed,
            default tolerance, defined by ``tol = SQRT( SQRT( EPS ) )``
            is used instead, as a relative tolerance, where `EPS` is
            the machine precision (see LAPACK Library routine DLAMCH).
            If `sort` = 'N' or 'C', this parameter is not referenced.

    Returns
    -------
        Ar : (n, n) ndarray
            Contains the computed block-diagonal matrix, in real Schur
            canonical form. The non-diagonal blocks are set to zero.
        Xr : (n, n) ndarray or None
            Contains the product of the given matrix `X` and the
            transformation matrix that reduced `A` to block-diagonal
            form. The transformation matrix is itself a product of
            non-orthogonal similarity transformations having elements
            with magnitude less than or equal to `pmax`.
            If `jobx` = 'N', this array is returned as None
        blsize : (n,) ndarray
            The orders of the resulting diagonal blocks of the matrix `Ar`.
        W : (n,) complex ndarray
            Contains the complex eigenvalues of the matrix `A`.

    Notes
    -----
    **Method**

    Consider first that `sort` = 'N'. Let

    ::

           ( A    A   )
           (  11   12 )
       A = (          ),
           ( 0    A   )
           (       22 )

    be the given matrix in real Schur form, where initially :math:`A_{11}` is the
    first diagonal block of dimension 1-by-1 or 2-by-2. An attempt is
    made to compute a transformation matrix `X` of the form

    ::

           ( I   P )
       X = (       )                                               (1)
           ( 0   I )

    (partitioned as `A`), so that

    ::

                ( A     0  )
        -1      (  11      )
       X  A X = (          ),
                ( 0    A   )
                (       22 )

    and the elements of `P` do not exceed the value `pmax` in magnitude.
    An adaptation of the standard method for solving Sylvester
    equations [1]_, which controls the magnitude of the individual
    elements of the computed solution [2]_, is used to obtain matrix `P`.
    When this attempt failed, an 1-by-1 (or 2-by-2) diagonal block of
    :math:`A_{22}`  , whose eigenvalue(s) is (are) the closest to the mean of those
    of :math:`A_{11}`   is selected, and moved by orthogonal similarity
    transformations in the leading position of :math:`A_{22}`  ; the moved diagonal
    block is then added to the block :math:`A_{11}`  , increasing its order by 1
    (or 2). Another attempt is made to compute a suitable
    transformation matrix X with the new definitions of the blocks :math:`A_{11}`
    and :math:`A_{22}`  . After a successful transformation matrix `X` has been
    obtained, it postmultiplies the current transformation matrix
    (if `jobx` = 'U'), and the whole procedure is repeated for the
    matrix :math:`A_{22}`.

    When `sort` = 'S', the diagonal blocks of the real Schur form are
    reordered before each step of the reduction, so that each cluster
    of eigenvalues, defined as specified in the definition of TOL,
    appears in adjacent blocks. The blocks for each cluster are merged
    together, and the procedure described above is applied to the
    larger blocks. Using the option `sort` = 'S' will usually provide
    better efficiency than the standard option (`sort` = 'N'), proposed
    in [2]_, because there could be no or few unsuccessful attempts
    to compute individual transformation matrices `X` of the form (1).
    However, the resulting dimensions of the blocks are usually
    larger; this could make subsequent calculations less efficient.

    When `sort` = 'C' or 'B', the procedure is similar to that for
    `sort` = 'N' or 'S', respectively, but the block of :math:`A_{22}` whose
    eigenvalue(s) is (are) the closest to those of :math:`A_{11}` (not to their
    mean) is selected and moved to the leading position of :math:`A_{22}`. This
    is called the "closest-neighbour" strategy.

    **Numerical Aspects**

    The algorithm usually requires :math:`\\mathcal{O}(N^3)` operations,
    but :math:`\\mathcal{O}(N^4)` are
    possible in the worst case, when all diagonal blocks in the real
    Schur form of `A` are 1-by-1, and the matrix cannot be diagonalized
    by well-conditioned transformations.

    **Further Comments**

    The individual non-orthogonal transformation matrices used in the
    reduction of `A` to a block-diagonal form have condition numbers
    of the order `pmax`*`pmax`. This does not guarantee that their product
    is well-conditioned enough. The routine can be easily modified to
    provide estimates for the condition numbers of the clusters of
    eigenvalues.

    **Contributor**

    V. Sima, Katholieke Univ. Leuven, Belgium, June 1998.
    Partly based on the RASP routine BDIAG by A. Varga, German
    Aerospace Center, DLR Oberpfaffenhofen.

    **Revisions**

    \\V. Sima, Research Institute for Informatics, Bucharest, Apr. 2003.

    References
    ----------
    .. [1] Bartels, R.H. and Stewart, G.W.  T
           Solution of the matrix equation A X + XB = C.
           Comm. A.C.M., 15, pp. 820-826, 1972.

    .. [2] Bavely, C. and Stewart, G.W.
           An Algorithm for Computing Reducing Subspaces by Block
           Diagonalization.
           SIAM J. Numer. Anal., 16, pp. 359-367, 1979.

    .. [3] Demmel, J.
           The Condition Number of Equivalence Transformations that
           Block Diagonalize Matrix Pencils.
           SIAM J. Numer. Anal., 20, pp. 599-610, 1983.

    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['jobx', 'sort', 'n', 'pmax',
                'A', 'lda' + hidden, 'X', 'ldx' + hidden,
                'nblcks', 'blsize', 'wr', 'wi', 'tol',
                'dwork' + hidden, 'info']

    if X is None:
        X = np.zeros((1, n))

    Ar, Xr, nblcks, blsize, wr, wi, info = _wrapper.mb03rd(
        jobx, sort, n, pmax, A, X, tol)

    raise_if_slycot_error(info, arg_list)
    if jobx == 'N':
        Xr = None
    else:
        Xr = Xr[:n, :n]
    Ar = Ar[:n, :n]
    W = wr + 1J*wi
    return Ar, Xr, blsize[:nblcks], W


def mb03vd(n, ilo, ihi, A):
    """ HQ, Tau = mb03vd(n, ilo, ihi, A)

    To reduce a product of p real general matrices A = A_1*A_2*...*A_p
    to upper Hessenberg form, H = H_1*H_2*...*H_p, where H_1 is
    upper Hessenberg, and H_2, ..., H_p are upper triangular, by using
    orthogonal similarity transformations on A,

            Q_1' * A_1 * Q_2 = H_1,
            Q_2' * A_2 * Q_3 = H_2,
                   ...
            Q_p' * A_p * Q_1 = H_p.

    Parameters
    ----------

    n : int
            The order of the square matrices A_1, A_2, ..., A_p.
            n >= 0.

    ilo, ihi : int
            It is assumed that all matrices A_j, j = 2, ..., p, are
            already upper triangular in rows and columns [:ilo-1] and
            [ihi:n], and A_1 is upper Hessenberg in rows and columns
            [:ilo-1] and [ihi:n], with A_1[ilo-1,ilo-2] = 0 (unless
            ilo = 1), and A_1[ihi,ihi-1] = 0 (unless ihi = n).
            If this is not the case, ilo and ihi should be set to 1
            and n, respectively.
            1 <= ilo <= max(1,n); min(ilo,n) <= ihi <= n.

    A : ndarray
            A[:n,:n,:p] must contain the matrices of factors to be reduced;
            specifically, A[:,:,j-1] must contain A_j, j = 1, ..., p.


    Returns
    -------

    HQ : ndarray
            3D array with same shape as A. The upper triangle and the first
            subdiagonal of HQ[:n,:n,0] contain the upper Hessenberg
            matrix H_1, and the elements below the first subdiagonal,
            with the first column of the array Tau represent the
            orthogonal matrix Q_1 as a product of elementary
            reflectors. See FURTHER COMMENTS.
            For j > 1, the upper triangle of HQ[:n,:n,j-1]
            contains the upper triangular matrix H_j, and the elements
            below the diagonal, with the j-th column of the array TAU
            represent the orthogonal matrix Q_j as a product of
            elementary reflectors. See FURTHER COMMENTS.

    Tau : ndarray
            2D array with shape (max(1, n-1), p).
            The leading n-1 elements in the j-th column contain the
            scalar factors of the elementary reflectors used to form
            the matrix Q_j, j = 1, ..., p. See FURTHER COMMENTS.

    Further Comments
    ----------------

    Each matrix Q_j is represented as a product of (ihi-ilo)
    elementary reflectors,

       Q_j = H_j(ilo) H_j(ilo+1) . . . H_j(ihi-1).

    Each H_j(i), i = ilo, ..., ihi-1, has the form

       H_j(i) = I - tau_j * v_j * v_j',

    where tau_j is a real scalar, and v_j is a real vector with
    v_j(1:i) = 0, v_j(i+1) = 1 and v_j(ihi+1:n) = 0; v_j(i+2:ihi)
    is stored on exit in A_j(i+2:ihi,i), and tau_j in TAU(i,j).

    The contents of A_1 are illustrated by the following example
    for n = 7, ilo = 2, and ihi = 6:

    on entry                         on exit

    ( a   a   a   a   a   a   a )    ( a   h   h   h   h   h   a )
    ( 0   a   a   a   a   a   a )    ( 0   h   h   h   h   h   a )
    ( 0   a   a   a   a   a   a )    ( 0   h   h   h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  h   h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  v3  h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  v3  v4  h   h   h )
    ( 0   0   0   0   0   0   a )    ( 0   0   0   0   0   0   a )

    where a denotes an element of the original matrix A_1, h denotes
    a modified element of the upper Hessenberg matrix H_1, and vi
    denotes an element of the vector defining H_1(i).

    The contents of A_j, j > 1, are illustrated by the following
    example for n = 7, ilo = 2, and ihi = 6:

    on entry                         on exit

    ( a   a   a   a   a   a   a )    ( a   h   h   h   h   h   a )
    ( 0   a   a   a   a   a   a )    ( 0   h   h   h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  h   h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  v3  h   h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  v3  v4  h   h   h )
    ( 0   a   a   a   a   a   a )    ( 0   v2  v3  v4  v5  h   h )
    ( 0   0   0   0   0   0   a )    ( 0   0   0   0   0   0   a )

    where a denotes an element of the original matrix A_j, h denotes
    a modified element of the upper triangular matrix H_j, and vi
    denotes an element of the vector defining H_j(i). (The element
    (1,2) in A_p is also unchanged for this example.)

    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['n', 'p' + hidden,
                'ilo', 'ihi', 'a',
                'lda1' + hidden, 'lda2' + hidden, 'tau',
                'ldtau' + hidden, 'dwork' + hidden, 'info']

    HQ, Tau, info = _wrapper.mb03vd(n, ilo, ihi, A)

    raise_if_slycot_error(info, arg_list)
    return (HQ, Tau)


def mb03vy(n, ilo, ihi, A, Tau, ldwork=None):
    """ Q = mb03vy(n, ilo, ihi, A, Tau, [ldwork])

    To generate the real orthogonal matrices Q_1, Q_2, ..., Q_p,
    which are defined as the product of ihi-ilo elementary reflectors
    of order n, as returned by SLICOT Library routine MB03VD:

        Q_j = H_j(ilo) H_j(ilo+1) . . . H_j(ihi-1).

    Parameters
    ----------

    n : int
            The order of the matrices Q_1, Q_2, ..., Q_p.  N >= 0.

    ilo, ihi : int
            The values of the indices ilo and ihi, respectively, used
            in the previous call of the SLICOT Library routine MB03VD.
            1 <= ilo <= max(1,n); min(ilo,n) <= ihi <= n.

    A : ndarray
            A[:n,:n,j-1] must contain the vectors which define the
            elementary reflectors used for reducing A_j, as returned
            by SLICOT Library routine MB03VD, j = 1, ..., p.

    Tau : ndarray
            The leading N-1 elements in the j-th column must contain
            the scalar factors of the elementary reflectors used to
            form the matrix Q_j, as returned by SLICOT Library routine
            MB03VD.

    ldwork : int, optional
            The length of the internal array DWORK.  ldwork >= max(1, n).
            For optimum performance ldwork should be larger.


    Returns
    -------

    Q : ndarray
            3D array with same shape as A. Q[:n,:n,j-1] contains the
            N-by-N orthogonal matrix Q_j, j = 1, ..., p.

    """

    hidden = ' (hidden by the wrapper)'
    arg_list = ['n', 'p' + hidden,
                'ilo', 'ihi', 'a',
                'lda1' + hidden, 'lda2' + hidden, 'tau',
                'ldtau' + hidden, 'dwork' + hidden, 'ldwork', 'info' + hidden]

    if not ldwork:
        ldwork = max(1, 2 * n)

    Q, info = _wrapper.mb03vy(n, ilo, ihi, A, Tau, ldwork)

    raise_if_slycot_error(info, arg_list)

    return Q


def mb03wd(job, compz, n, ilo, ihi, iloz, ihiz, H, Q, ldwork=None):
    """ T, Z, Wr = mb03wd(job, compz, n, ilo, ihi, iloz, ihiz, H, Q, [ldwork])

    To compute the Schur decomposition and the eigenvalues of a
    product of matrices, H = H_1*H_2*...*H_p, with H_1 an upper
    Hessenberg matrix and H_2, ..., H_p upper triangular matrices,
    without evaluating the product. Specifically, the matrices Z_i
    are computed, such that

    ::

            Z_1' * H_1 * Z_2 = T_1,
            Z_2' * H_2 * Z_3 = T_2,
                   ...
            Z_p' * H_p * Z_1 = T_p,

    where T_1 is in real Schur form, and T_2, ..., T_p are upper
    triangular.

    The routine works primarily with the Hessenberg and triangular
    submatrices in rows and columns ilo to ihi, but optionally applies
    the transformations to all the rows and columns of the matrices
    H_i, i = 1,...,p. The transformations can be optionally
    accumulated.

    Parameters
    ----------
    job : {'E', 'S'}
            Indicates whether the user wishes to compute the full
            Schur form or the eigenvalues only, as follows:
            = 'E':  Compute the eigenvalues only;
            = 'S':  Compute the factors T_1, ..., T_p of the full
                    Schur form, T = T_1*T_2*...*T_p.
   compz : {'N', 'I', 'V'}
            Indicates whether or not the user wishes to accumulate
            the matrices Z_1, ..., Z_p, as follows:
            = 'N':  The matrices Z_1, ..., Z_p are not required;
            = 'I':  Z_i is initialized to the unit matrix and the
                    orthogonal transformation matrix Z_i is returned,
                    i = 1, ..., p;
            = 'V':  Z_i must contain an orthogonal matrix Q_i on
                    entry, and the product Q_i*Z_i is returned,
                    i = 1, ..., p.
    n : int
            The order of the matrix H.  n >= 0
    ilo, ihi : int
            It is assumed that all matrices H_j, j = 2, ..., p, are
            already upper triangular in rows and columns [:ilo-1] and
            [ihi:n], and H_1 is upper quasi-triangular in rows and
            columns [:ilo-1] and [ihi:n], with H_1[ilo-1,ilo-2] = 0
            (unless ilo = 1), and H_1[ihi,ihi-1] = 0 (unless ihi = n).
            The routine works primarily with the Hessenberg submatrix
            in rows and columns ilo to ihi, but applies the
            transformations to all the rows and columns of the
            matrices H_i, i = 1,...,p, if JOB = 'S'.
            1 <= ilo <= max(1,n); min(ilo,n) <= ihi <= n.
    iloz, ihiz : int
            Specify the rows of Z to which the transformations must be
            applied if compz = 'I' or compz = 'V'.
            1 <= iloz <= ilo; ihi <= ihiz <= n.
    H : array_like
            H[:n,:n,0] must contain the upper Hessenberg matrix H_1 and
            H[:n,:n,j-1] for j > 1 must contain the upper triangular matrix
            H_j, j = 2, ..., p.
    Q : array_like
            If compz = 'V', Q[:n,:n,:p] must contain the current matrix Q of
            transformations accumulated by SLICOT Library routine
            MB03VY.
            If compz = 'I', Q is ignored
    ldwork : int, optional
            The length of the cache array. The default value is
            ihi-ilo+p-1

    Returns
    -------
    T : ndarray
            3D array with the same shape as H.
            If JOB = 'S', T[:n,:n,0] is upper quasi-triangular in rows
            and columns [ilo-1:ihi], with any 2-by-2 diagonal blocks
            corresponding to a pair of complex conjugated eigenvalues, and
            T[:n,:n,j-1] for j > 1 contains the resulting upper
            triangular matrix T_j.
            If job = 'E', T is None
    Z : ndarray
            3D array with the same shape as Q.
            If compz = 'V', or compz = 'I', the leading
            N-by-N-by-P part of this array contains the transformation
            matrices which produced the Schur form; the
            transformations are applied only to the submatrices
            Z[iloz-1:ihiz,ilo-1:ihi,j-1], j = 1, ..., p.
            If compz = 'N', Z is None
    W : (n,) complex ndarray
            The computed eigenvalues ilo to ihi. If two eigenvalues
            are computed as a complex conjugate pair, they are stored
            in consecutive elements of W say the i-th and
            (i+1)th, with imag(W][i]) > 0 and imag(W[i+1]) < 0.
            If JOB = 'S', the eigenvalues are stored in the same order
            as on the diagonal of the Schur form returned in H.

    Warns
    -----
    SlycotResultWarning
        :info > 0:
            failed to compute all the eigenvalues {ilo} to {ihi}
            in a total of 30*({ihi}-{ilo}+1) iterations
            the elements Wr[{info}:{ihi}] contains those
            eigenvalues which have been successfully computed.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['job', 'compz', 'n', 'p' + hidden,
                'ilo', 'ihi', 'iloz', 'ihiz',
                'h', 'ldh1' + hidden, 'ldh2' + hidden,
                'z', 'ldz1' + hidden, 'ldz2' + hidden,
                'wr', 'wi',
                'dwork' + hidden, 'ldwork', 'info' + hidden]

    if not ldwork:
        p = H.shape[2]
        ldwork = max(1, ihi - ilo + p - 1)

    T, Z, Wr, Wi, info = _wrapper.mb03wd(
            job, compz, n, ilo, ihi, iloz, ihiz, H, Q, ldwork)
    raise_if_slycot_error(info, arg_list, mb03rd.__doc__, locals())

    if job == 'E':
        T = None
    if compz == 'N':
        Z = None

    W = Wr + Wi*1J
    return (T, Z, W)


def mb05md(a, delta, balanc='N'):
    """Ar, Vr, Yr, w = mb05md(a, delta, balanc='N')

    Matrix exponential for a real non-defective matrix

    To compute ``exp(A*delta)`` where `A` is a real N-by-N non-defective
    matrix with real or complex eigenvalues and delta is a scalar
    value. The routine also returns the eigenvalues and eigenvectors
    of `A` as well as (if all eigenvalues are real) the matrix product
    ``exp(Lambda*delta)`` times the inverse of the eigenvector matrix of
    `A`, where `Lambda` is the diagonal matrix of eigenvalues.
    Optionally, the routine computes a balancing transformation to
    improve the conditioning of the eigenvalues and eigenvectors.

    Parameters
    ----------
    A : (n, n) array_like
        Square matrix
    delta : float
        The scalar value delta of the problem.
    balanc : {'N', 'S'}, optional
        Indicates how the input matrix should be diagonally scaled
        to improve the conditioning of its eigenvalues as follows:

        := 'N':  Do not diagonally scale;
        := 'S':  Diagonally scale the matrix, i.e. replace `A` by
                 ``D*A*D**(-1)``, where `D` is a diagonal matrix chosen
                 to make the rows and columns of A more equal in
                 norm. Do not permute.

    Returns
    -------
    Ar : (n, n) ndarray
        Contains the solution matrix ``exp(A*delta)``
    Vr : (n, n) ndarray
        Contains the eigenvector matrix for `A`.  If the `k`-th
        eigenvalue is real the `k`-th column of the eigenvector
        matrix holds the eigenvector corresponding to the `k`-th
        eigenvalue.  Otherwise, the `k`-th and `(k+1)`-th eigenvalues
        form a complex conjugate pair and the k-th and `(k+1)`-th
        columns of the eigenvector matrix hold the real and
        imaginary parts of the eigenvectors corresponding to these
        eigenvalues as follows.  If `p` and `q` denote the `k`-th and
        `(k+1)`-th columns of the eigenvector matrix, respectively,
        then the eigenvector corresponding to the complex
        eigenvalue with positive (negative) imaginary value is
        given by
          ``p + q*j (p - q*j), where j^2  = -1.``
    Yr : (n, n) ndarray
        contains an intermediate result for computing the matrix
        exponential.  Specifically, ``exp(A*delta)`` is obtained as the
        product ``V*Y``, where `V` is the matrix stored in the leading
        `n`-by-`n` part of the array `V`. If all eigenvalues of `A` are
        real, then the leading `n`-by-`n` part of this array contains
        the matrix product ``exp(Lambda*delta)`` times the inverse of
        the (right) eigenvector matrix of `A`, where `Lambda` is the
        diagonal matrix of eigenvalues.
    w : (n, ) real or complex ndarray
        Contains the eigenvalues of the matrix `A`. The eigenvalues
        are unordered except that complex conjugate pairs of values
        appear consecutively with the eigenvalue having positive
        imaginary part first.

    Warns
    ------
    SlycotResultWarning
        :0 < info <=n:
            the QR algorithm failed to compute all
            the eigenvalues; no eigenvectors have been computed;
            w[{info}:{n}] contains eigenvalues which have converged;
        :info == n+1:
            The inverse of the eigenvector matrix could not
            be formed due to an attempt to divide by zero, i.e.,
            the eigenvector matrix is singular;
        :info == n+2:
            Matrix A is defective, possibly due to rounding errors.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['balanc', 'n', 'delta', 'a', 'lda'+hidden, 'v', 'ldv'+hidden,
                'y', 'ldy'+hidden, 'valr', 'vali',
                'iwork'+hidden, 'dwork'+hidden, 'ldwork'+hidden,
                'info'+hidden]
    n = min(a.shape)
    (Ar, Vr, Yr, VALr, VALi, INFO) = _wrapper.mb05md(balanc=balanc,
                                                     n=n,
                                                     delta=delta,
                                                     a=a)
    raise_if_slycot_error(INFO, arg_list, mb05md.__doc__, locals())

    if not all(VALi == 0):
        w = VALr + 1J*VALi
    else:
        w = VALr
    return (Ar, Vr, Yr, w)



def mb05nd(a, delta, tol=1e-7):
    """F, H = mb05nd(n, a, delta, tol=1e-7)

    To compute

    ::

     (a)    F(delta) =  exp(A*delta) and
     (b)    H(delta) =  Int[F(s) ds] from s = 0 to s = delta,

    where `A` is a real`n`-by-`n` matrix and `delta` is a scalar value.

    Parameters
    ----------
    A : (n, n) array_like
        Square matrix
    delta : float
        The scalar value delta of the problem.
    tol : float
        Tolerance. A good value is sqrt(eps)

    Returns
    -------
    F : (n, n) ndarray
        exp(A*delta)
    H : (n, n) ndarray
        Int[F(s) ds] from s = 0 to s = delta,

    Raises
    ------
    SlycotArithmeticError
        :1 < info <=n:
            the ({info},{info}) element of the denominator of
            the Pade approximation is zero, so the denominator
            is exactly singular;
        :info == n+1:
            ``DELTA = (delta * frobenius norm of matrix A)`` is
            probably too large to permit meaningful computation.
            That is, {delta} > SQRT(BIG), where BIG is a
            representable number near the overflow threshold of
            the machine (see LAPACK Library Routine DLAMCH).
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['n', 'delta', 'a', 'lda'+hidden, 'ex', 'ldex'+hidden,
                'exint', 'ldexin'+hidden, 'tol', 'iwork'+hidden,
                'dwork'+hidden, 'ldwork'+hidden]
    n = min(a.shape)
    out = _wrapper.mb05nd(n=n, delta=delta, a=a, tol=tol)
    raise_if_slycot_error(out[-1], arg_list, mb05nd.__doc__, locals())
    return out[:-1]



def mc01td(dico, dp, p):
    """dp, stable, nz = mc01td(dico, dp, p)

    To determine whether or not a given polynomial P(x) with real
    coefficients is stable, either in the continuous-time or discrete-
    time case.

    A polynomial is said to be stable in the continuous-time case
    if all its zeros lie in the left half-plane, and stable in the
    discrete-time case if all its zeros lie inside the unit circle.


    Parameters
    ----------
    dico : {'C', 'D'}
        Indicates whether the stability test to be applied to `P(x)` is in
        the continuous-time or discrete-time case as follows::

        = 'C':  continuous-time case;
        = 'D':  discrete-time case.

    dp : int
        The degree of the polynomial `P(x)`.  ``dp >= 0``.
    p : (dp+1, ) array_like
        This array must contain the coefficients of `P(x)` in increasing
        powers of `x`.

    Returns
    -------
    dp : int
        If ``P(dp+1) = 0.0`` on entry, then `dp` contains the index of the
        highest power of `x` for which ``P(dp+1) <> 0.0``.
    stable : int
        Equal to 1 if `P(x)` is stable, 0 otherwise.
    nz : int
        The number of unstable zeros.

    Warns
    -----
    SlycotResultWarning
        :info == 1:
            Entry ``P(x)`` is the zero polynomial.
        :info == 2 and dico == 'C':
            The polynomial ``P(x)`` is most probably unstable,
            although it may be stable with one or more zeros
            very close to the imaginary axis.
            The number of unstable zeros (NZ) is not determined.
        :info == 2 and dico == 'D':
            The polynomial ``P(x)`` is most probably unstable,
            although it may be stable with one or more zeros
            very close to the the unit circle.
            The number of unstable zeros (NZ) is not determined.
        :iwarn > 0:
            The degree of the polynomial ``P(x)`` has been
            reduced to ``(DB - {iwarn})`` because
            ``P(DB+1-j) = 0.0`` on entry
            for ``j = 0, 1,..., k-1`` and ``P(DB+1-k) <> 0.0``.
    """
    hidden = ' (hidden by the wrapper)'
    arg_list = ['dico', 'dp', 'P', 'stable', 'nz', 'DWORK' + hidden,
                'IWARN', 'INFO']
    (dp_out, stable_log, nz, iwarn, info) = _wrapper.mc01td(dico, dp, p)
    raise_if_slycot_error([iwarn, info], arg_list, mc01td.__doc__, locals())
    ftrue, ffalse = _wrapper.ftruefalse()
    stable = 1 if stable_log == ftrue else 0
    return (dp_out, stable, nz)
